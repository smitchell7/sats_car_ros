
#include "timing.h"

#include "VP_tm4c123gh6pm.h"

//*****************************************************************************
//Interrupt priorities. High priority equates to low numbers. 
/*! 
	\def TIMER0_PRIORITY 
	Priority of the main control interrupt. High priority equates to low numbers. 
	\def TIMER1_PRIORITY
	Priority of the UART transmit interrupt. High priority equates to low numbers.
	\def TIMER2_PRIORITY
	Priority of the Lidar read interrupt. High priority equates to low numbers. 
	\def UART1_PRIORITY 
	Priority of the UART read interrupt. High priority equates to low numbers. 
*///*****************************************************************************
#define TIMER0_PRIORITY 0x2 // High level 
#define TIMER1_PRIORITY 0x1 // Transmit data
#define TIMER3_PRIORITY 0x3 // Low level
#define XBEE_UART_PRIORITY  0x0 // Receive data
#define I2CMAN_PRIORITY   0x0 // Receive data
#define QEI_PRIORITY    0x1

void timing_wait(float seconds)
{
	uint32_t loadval = (float)SysCtlClockGet()*seconds;
	TimerLoadSet(TIMER2_BASE, TIMER_A, loadval);
	TimerEnable(TIMER2_BASE, TIMER_A);
	while(TimerValueGet(TIMER2_BASE, TIMER_A) != loadval);
}

void timing_init(const timing_cb_list_t * cblist)
{
	//****************************************************************************
	// Enable and configure the timers
	//
	// Timer0 is used for the Timer0DelayUntil functions, the reload value is set
	// there.
	//****************************************************************************
	//Enable the peripherals used by the timers
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
	
	//****************************************************************************
	//Configure the 32-bit periodic Timer0
	//****************************************************************************
	//TimerConfigure disables the timer before setting parameters. The timer is left
	//dissabled here
  /*
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	uint32_t timer_load_value = (uint32_t)((float)SysCtlClockGet()*HIGH_LEVEL_TIME);
	TimerDisable(TIMER0_BASE, TIMER_A);
	TimerLoadSet(TIMER0_BASE, TIMER_A, timer_load_value);
	TimerEnable(TIMER0_BASE, TIMER_A);
  */
	
	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
	uint32_t timer_load_value = (uint32_t)((float)SysCtlClockGet()*LOG_TRANSMIT_TIME);
	TimerDisable(TIMER1_BASE, TIMER_A);
	TimerLoadSet(TIMER1_BASE, TIMER_A, timer_load_value);
	TimerEnable(TIMER1_BASE, TIMER_A);
	
	TimerConfigure(TIMER2_BASE, TIMER_CFG_ONE_SHOT);
	TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
	
  /*
	TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
	timer_load_value = (uint32_t)((float)SysCtlClockGet()*LOW_LEVEL_TIME);
	TimerDisable(TIMER3_BASE, TIMER_A);
	TimerLoadSet(TIMER3_BASE, TIMER_A, timer_load_value);
	TimerEnable(TIMER3_BASE, TIMER_A);
  */
	
	// Setup the interrupt for the Timer0 timeout
  /*
	IntEnable(INT_TIMER0A);
	IntRegister(INT_TIMER0A, cblist->high_level_controller);
	IntPrioritySet(INT_TIMER0A, TIMER0_PRIORITY);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  */
	
	// Setup the interrupt for the Timer3 timeout
  /*
	IntEnable(INT_TIMER3A);
	IntRegister(INT_TIMER3A, cblist->low_level_controller);
	IntPrioritySet(INT_TIMER3A, TIMER3_PRIORITY);
	TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
  */
	
	// Setup the interrupt for the Timer1 timeout
	IntEnable(INT_TIMER1A);
	IntRegister(INT_TIMER1A, cblist->log_function);
	IntPrioritySet(INT_TIMER1A, TIMER1_PRIORITY);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	
	// High priority comes with low numbers
	//IntRegister(cblist->xbee_uart.interrupt, cblist->xbee_uart.handler);
	IntRegister(cblist->i2cman.interrupt, cblist->i2cman.handler);
	IntRegister(cblist->qei_left.interrupt, cblist->qei_left.handler);
	IntRegister(cblist->qei_right.interrupt, cblist->qei_right.handler);
	
	//IntPrioritySet(cblist->xbee_uart.interrupt, XBEE_UART_PRIORITY);
	IntPrioritySet(cblist->i2cman.interrupt, I2CMAN_PRIORITY);
	IntPrioritySet(cblist->qei_left.interrupt, QEI_PRIORITY);
	IntPrioritySet(cblist->qei_right.interrupt, QEI_PRIORITY);
}
