
#include "i2cman.h"

#define I2C_PORT I2C0_BASE

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_i2c.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/rom.h"

static int finish_op(const i2c_op_t * op)
{
	// Check for errors first
	uint32_t error_status = ROM_I2CMasterErr(I2C_PORT);
	if(error_status != I2C_MASTER_ERR_NONE)
	{
		return error_status; // return with errors
	}
  
	switch(op->op)
	{
		case OP_BURST_RECEIVE_START:
		case OP_BURST_RECEIVE_CONT:
		case OP_BURST_RECEIVE_FINISH:
		case OP_SINGLE_RECEIVE:
			*op->valueptr = ROM_I2CMasterDataGet(I2C_PORT);
		case OP_BURST_SEND_START:
		case OP_BURST_SEND_CONT:
		case OP_BURST_SEND_FINISH:
		case OP_SINGLE_SEND:
			break;
	}
  
	return I2C_MASTER_ERR_NONE; // no errors
}
static void start_op(const i2c_op_t * op)
{
	switch(op->op)
	{
		case OP_SINGLE_SEND:
		case OP_BURST_SEND_START:
			// set slave address when we start
			ROM_I2CMasterSlaveAddrSet(I2C_PORT, op->address, false);
		case OP_BURST_SEND_CONT:
		case OP_BURST_SEND_FINISH:
			// always put data for a send
			ROM_I2CMasterDataPut(I2C_PORT, op->value);
			break;

		case OP_SINGLE_RECEIVE:
		case OP_BURST_RECEIVE_START:
			// set slave address when we start
			ROM_I2CMasterSlaveAddrSet(I2C_PORT, op->address, true);
		case OP_BURST_RECEIVE_CONT:
		case OP_BURST_RECEIVE_FINISH:
			break;
	}
	uint32_t master_cmd = 0;
	switch(op->op)
	{
		case OP_SINGLE_SEND:
			master_cmd = I2C_MASTER_CMD_SINGLE_SEND;
			break;
		case OP_BURST_SEND_START:
			master_cmd = I2C_MASTER_CMD_BURST_SEND_START;
			break;
		case OP_BURST_SEND_CONT:
			master_cmd = I2C_MASTER_CMD_BURST_SEND_CONT;
			break;
		case OP_BURST_SEND_FINISH:
			master_cmd = I2C_MASTER_CMD_BURST_SEND_FINISH;
			break;
		case OP_SINGLE_RECEIVE:
			master_cmd = I2C_MASTER_CMD_SINGLE_RECEIVE;
			break;
		case OP_BURST_RECEIVE_START:
			master_cmd = I2C_MASTER_CMD_BURST_RECEIVE_START;
			break;
		case OP_BURST_RECEIVE_CONT:
			master_cmd = I2C_MASTER_CMD_BURST_RECEIVE_CONT;
			break;
		case OP_BURST_RECEIVE_FINISH:
			master_cmd = I2C_MASTER_CMD_BURST_RECEIVE_FINISH;
			break;
	}
	ROM_I2CMasterControl(I2C_PORT, master_cmd);
}
static void kill_op(const i2c_op_t * op)
{
	uint32_t master_cmd = 0;
	switch(op->op)
	{
		case OP_BURST_SEND_START:
		case OP_BURST_SEND_CONT:
		case OP_BURST_SEND_FINISH:
			master_cmd = I2C_MASTER_CMD_BURST_SEND_ERROR_STOP;
		case OP_SINGLE_SEND:
			break;
		case OP_BURST_RECEIVE_START:
		case OP_BURST_RECEIVE_CONT:
		case OP_BURST_RECEIVE_FINISH:
			master_cmd = I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP;
		case OP_SINGLE_RECEIVE:
			break;
	}
	ROM_I2CMasterControl(I2C_PORT, master_cmd);
}

const i2c_op_t * current_op = 0;
const i2c_op_t * begin = 0;
const i2c_op_t * end = 0;
void (* success_func)(void);
void (* error_func)(uint32_t);

static void stop()
{
	begin = 0;
	end = 0;
	current_op = 0;
}
void i2cman_handler()
{
	ROM_I2CMasterIntClear(I2C_PORT);
	
	if(current_op == end || current_op == 0)
		return; // do absolutely nothing (irq may be called after aborting)
	
	uint32_t error = finish_op(current_op);
	if(error != I2C_MASTER_ERR_NONE)
	{
		stop();
		if(error & I2C_MASTER_ERR_ARB_LOST)
		{
			if(error_func)
				error_func(I2C_ARBLOST);
		}
		else
		{
			// error, abort completely
			//kill_op(current_op);
			// bad bad bad!
			if(error_func)
				error_func(I2C_NOACK);
		}
		return;
	}
	
	current_op ++;
	
	if(current_op == end)
	{
		stop(); // all done
		if(success_func)
			success_func();
		return;
	}
	
	// otherwise, process the next op
	start_op(current_op);
}

void i2cman_abort()
{
	if(current_op)
		kill_op(current_op);
	stop();
}
bool i2cman_execute(const i2c_op_t * _begin, const i2c_op_t * _end, void (* succ)(void), void (* err)(uint32_t))
{
	if(!current_op)
	{
		begin = _begin;
		end = _end;
		current_op = begin;
		success_func = succ;
		error_func = err;
		
		// activates the interrupt handler
		//
		// also activates my trap card
		start_op(current_op);
		
		return true;
	}
	
	return false;
}
void i2cman_init()
{
	//Enable I2C0 which by default uses PortB[3:2] for SDA and SCL respectively
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
	
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);
	
	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
	
	I2CMasterInitExpClk(I2C_PORT, SysCtlClockGet(), false);
	I2CMasterGlitchFilterConfigSet(I2C_PORT, I2C_MASTER_GLITCH_FILTER_32);
	
	ROM_I2CMasterIntEnable(I2C_PORT);
	
	IntEnable(INT_I2C0);
}
