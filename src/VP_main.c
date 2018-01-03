/**
 *
 * !\file VP_main.c - Main loop
 *
 * \author David Petrizze
 *
 * \date February 2016
 *
 * \brief Car code has been gutted, repurposed as a sensor hub
 */

#include "VP_tm4c123gh6pm.h"
#include "i2cman.h"
#include "lidar.h"
#include "rajamani.h"
#include "qei.h"
#include "timing.h"

#include <assert.h>

// Non global variables at global scope must be declared static, and
// belong in a .c file

#define GEAR_RATIO 3.4            
#define WHEEL_CIRCUMFERENCE 0.319024

void log_init()
{
  //****************************************************************************
	//Configure UART1: Xbee
	//****************************************************************************
	//Enable the GPIO Peripheral used by UART1.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	//Enable UART1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	//Configure GPIO Pins for UART mode.
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	//Use the internal 16MHz oscillator as the UART clock source.
	UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
	//Enable the FIFO. 
	UARTFIFOEnable(UART1_BASE);
	UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX1_8, UART_FIFO_RX7_8);
	//Initialize the UART for console I/O.
	//UARTStdioConfig(1, 115200, 16000000);
	UARTConfigSetExpClk(
		UART1_BASE, 
		16000000, 
		460800, 
		UART_CONFIG_WLEN_8 |
		UART_CONFIG_STOP_ONE |
		UART_CONFIG_PAR_NONE);
}
void heartbeat_log(void)
{
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  float datas[10] = {
		_frontlidar_.distance,
		_frontlidar_.velocity,
		_frontlidar_.acceleration,
		_qei_leftmotor_.distance,
		_qei_leftmotor_.velocity,
		_qei_leftmotor_.acceleration,
		_qei_rightmotor_.distance,
		_qei_rightmotor_.velocity,
		_qei_rightmotor_.acceleration,
    0.0f
	};
  
  uint8_t * bytes = (uint8_t *)datas;
  int nbytes = sizeof(datas);
  
  UARTCharPut(UART1_BASE, 0x7E);
  uint8_t sum = 0x7E;
  for(int i = 0 ; i < nbytes ; i ++) {
    UARTCharPut(UART1_BASE, bytes[i]);
    sum += bytes[i];
  }
  UARTCharPut(UART1_BASE, -sum);
}

void HardFault_Handler()
{
	//motors_setpwm(0, 0);
}

#ifdef DEBUG
///\brief Called by the driver library if it encounters an error.
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

// Initialize software modules
//
// Software modules build abstractions on top of zero or more
// hardware peripherals.
//
// Software modules internally shall be configuration agnostic.
// For example, whether the vehicle is high performance or regular
// shall have absolutely no influence on the contents of the module.
// Rather, variables like that must be configured here.
//
// Main should be thought of as the glue that holds modules together.

int main(void)
{
	IntMasterDisable();
  
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	FPUEnable();
	FPUStackingEnable();
	FPULazyStackingEnable();
	
	// Setup the system clock to run at 20 Mhz from PLL with crystal reference
	SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
	
	// Just double check that the lidar knows how fast it's sampling
	assert(HIGH_LEVEL_TIME == LIDAR_SAMPLE_TIME);
  
  log_init();
	lidar_init();
	qei_init(GEAR_RATIO, WHEEL_CIRCUMFERENCE);
	
	const timing_cb_list_t cblist = {
		.log_function = heartbeat_log,
		.i2cman = { I2CMAN_INTERRUPT, i2cman_handler },
		.qei_right = { QEI_RIGHT_INTERRUPT, qei_right_handler },
		.qei_left = { QEI_LEFT_INTERRUPT, qei_left_handler },
	};
	
	timing_init(&cblist);
	
	IntMasterEnable();
	
	while (1) {}
}
