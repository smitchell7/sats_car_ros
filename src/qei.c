
#include "qei.h"
#include "FIRFilter.h"

#include "VP_tm4c123gh6pm.h"

static float count_calibration = 0;

// assumed by the internal velocity calculations, timers run this fast
#define QEI_SAMPLE_PERIOD 0.025

/*****************************************************************************
	\def ENCODER_PPR 
	Number of QEI pulses per revolution. 
*///*****************************************************************************
#define ENCODER_PPR 2000.0

/*
void qei_read(void)
{
	V->right_velocity = (float)E->right_raw_vel * E->vel_calibration;
	V->left_velocity = (float)E->left_raw_vel * E->vel_calibration;
	
	V->velocity = (V->right_velocity + V->left_velocity) / 2.0;
	
	//if (V->velocity == 0)
	//	V->velocity = 0.0001;
	
	V->k = (2/VEHICLE_WIDTH)*(V->right_velocity - V->left_velocity)/(V->right_velocity + V->left_velocity);
	
	V->heading += (V->right_velocity - V->left_velocity) * s_t / (2.0 * VEHICLE_WIDTH);
	V->heading = fmod(V->heading, 6.283185);
	//V->x += V->velocity * cos(V->heading) * SAMPLE_TIME;
	V->y += V->velocity * sin(V->heading) * s_t;
	
	V->x += V->velocity * s_t;
}
*/

qei_t _qei_rightmotor_ = {0};
//static FD4 right_stov = FD4_INIT;
//static FD4 right_vtoa = FD4_INIT;

// Right wheel
void qei_right_handler()
{
	QEIIntClear(QEI0_BASE, QEI_INTTIMER);
	
	float prev_x = _qei_rightmotor_.distance;
	float prev_v = _qei_rightmotor_.velocity;

	int right_raw_delta = QEI0_SPEED_R;
	if(QEIDirectionGet(QEI0_BASE) > 0)
		_qei_rightmotor_.distance += count_calibration * right_raw_delta;
	else
		_qei_rightmotor_.distance -= count_calibration * right_raw_delta;
	
	_qei_rightmotor_.velocity = (_qei_rightmotor_.distance - prev_x)/QEI_SAMPLE_PERIOD;
	_qei_rightmotor_.acceleration = (_qei_rightmotor_.velocity - prev_v)/ (2.0f * QEI_SAMPLE_PERIOD ) ;
	//_qei_rightmotor_.velocity = FD4_flush(&right_stov, _qei_rightmotor_.distance, QEI_SAMPLE_PERIOD);
	//_qei_rightmotor_.acceleration = FD4_flush(&right_vtoa, _qei_rightmotor_.velocity, QEI_SAMPLE_PERIOD);
}

qei_t _qei_leftmotor_ = {0};
//static FD4 left_stov = FD4_INIT;
//static FD4 left_vtoa = FD4_INIT;

// Left wheel
void qei_left_handler()
{
	QEIIntClear(QEI1_BASE, QEI_INTTIMER);
	
	float prev_x = _qei_leftmotor_.distance;
	float prev_v = _qei_leftmotor_.velocity;

	int left_raw_delta = QEI1_SPEED_R;
	if(QEIDirectionGet(QEI1_BASE) > 0)
		_qei_leftmotor_.distance += count_calibration * left_raw_delta;
	else
		_qei_leftmotor_.distance -= count_calibration * left_raw_delta;
	
	_qei_leftmotor_.velocity = (_qei_leftmotor_.distance - prev_x)/QEI_SAMPLE_PERIOD;
	_qei_leftmotor_.acceleration = (_qei_leftmotor_.velocity - prev_v)/ (2.0f * QEI_SAMPLE_PERIOD );	
	//_qei_leftmotor_.velocity = FD4_flush(&left_stov, _qei_leftmotor_.distance, QEI_SAMPLE_PERIOD);
	//_qei_leftmotor_.acceleration = FD4_flush(&left_vtoa, _qei_leftmotor_.velocity, QEI_SAMPLE_PERIOD);
}
void qei_recalibrate(float gear_ratio, float wheel_circumference)
{
	count_calibration = (1 / (float)ENCODER_PPR) * (1 / gear_ratio) * wheel_circumference;
}

void qei_init(float gear_ratio, float wheel_circumference)
{
	count_calibration = (1 / (float)ENCODER_PPR) * (1 / gear_ratio) * wheel_circumference;

	//Enable QEI clocks
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
	//Enable GPIO clocks
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	//Configure the GPIO pin muxing to select QEI0 and QEI1 functions for these pins.
	//This step selects which alternate function is available for these pins.
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0xFF;
	HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = 0;

	GPIOPinConfigure(GPIO_PD6_PHA0);
	GPIOPinConfigure(GPIO_PD7_PHB0);
	GPIOPinConfigure(GPIO_PC5_PHA1);
	GPIOPinConfigure(GPIO_PC6_PHB1);

	//Configure the QEI function for these pins.
	GPIOPinTypeQEI(GPIO_PORTC_BASE, (GPIO_PIN_5 | GPIO_PIN_6));
	GPIOPinTypeQEI(GPIO_PORTD_BASE, (GPIO_PIN_6 | GPIO_PIN_7));

	//Configure the quadrature encoders to capture edges on both signals.
	//
	//No index is currently present.
	//
	//The max. count value is based off of 20 meters/sec (aprox. 45 mph)
	//20[m/s] / 0.318[m/rev] = 62.89[rev/sec], multiply this by 2000[count/rev]
	//to get 125786
	//
	QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET |
	                         QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 126000);
	QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET |
	                         QEI_CONFIG_QUADRATURE | QEI_CONFIG_SWAP), 126000);
	
	//Configure QEI velocity output, sample at the system sample time
	QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, (SysCtlClockGet()*QEI_SAMPLE_PERIOD));
	QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1, (SysCtlClockGet()*QEI_SAMPLE_PERIOD));
	
	//Enable QEI velocity output
	QEIVelocityEnable(QEI0_BASE);
	QEIVelocityEnable(QEI1_BASE);
	
	//Enable the quadrature encoders.
	QEIEnable(QEI0_BASE);
	QEIEnable(QEI1_BASE);
	
	IntEnable(INT_QEI0);
	IntEnable(INT_QEI1);
	
	//Enable the timer interrupt
	QEIIntEnable(QEI0_BASE, QEI_INTTIMER);
	QEIIntEnable(QEI1_BASE, QEI_INTTIMER);
}
