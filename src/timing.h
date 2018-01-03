#ifndef TIMING_H
#define TIMING_H

//*****************************************************************************
/*! 
	\def SAMPLE_TIME 
	System sample time in seconds
	\def IDLE_TRANSMIT_TIME 
	System UART communication time in seconds
*///*****************************************************************************
#define HIGH_LEVEL_TIME     0.025
#define LOW_LEVEL_TIME      (HIGH_LEVEL_TIME / 5.0)
#define LOG_TRANSMIT_TIME   0.002
#define LIDAR_MAX_READ      0.1

typedef void (* handler_t)(void);

typedef struct {
	unsigned int interrupt;
	handler_t handler;
} module_interrupt;

typedef struct {
	handler_t log_function;
	module_interrupt i2cman;
	module_interrupt qei_left;
	module_interrupt qei_right;
} timing_cb_list_t;

// high level delay function, should absolutely not
// be called during an interrupt
void timing_wait(float seconds);
void timing_init(const timing_cb_list_t * cblist);

#endif
