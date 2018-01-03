
#include "motors.h"

#include "VP_tm4c123gh6pm.h"

//*****************************************************************************
//PWM definitions
//*****************************************************************************
//Use system clock divided by two for PWM clock. Initialize PWM to 300Hz
/*!
	\def PWM_FREQUENCY  
	PWM frequency in Hz. 
	\def PWM_MIN_PULSE 
	PWM pulse that produces maximum negative acceleration
	\def PWM_BASE_PULSE 
	PWM pulse that produces 0 velocity
	\def PWM_MAX_PULSE  
	PWM pulse that produces maximum forward acceleration
	\def PWM_PULSE_DIFF 
	Difference from the base pulse to the maximum pulse. 
	\def PWM_SCALE      
*///*****************************************************************************
#define PWM_FREQUENCY  300
#define PWM_MIN_PULSE  10000
#define PWM_BASE_PULSE 15000
#define PWM_MAX_PULSE  20000
#define PWM_PULSE_DIFF 5000
#define PWM_SCALE      50.0 
#define PWM_HARDLIMIT  1.0f

typedef struct
{
	unsigned int port;
	float polarity;
} motor_t;

static motor_t left_motor = {
	.polarity = 1,
};
static motor_t right_motor = {
	.polarity = 1,
};

void motors_init(unsigned int polarity_l, unsigned int polarity_r)
{
	 left_motor.polarity = polarity_l == MOTOR_NORMAL ? 1 : -1;
	right_motor.polarity = polarity_r == MOTOR_NORMAL ? 1 : -1;
	
	//****************************************************************************
	//Configure PWM0
	//****************************************************************************
	//Set the PWM clock to the system clock divided by 2.
	SysCtlPWMClockSet(SYSCTL_PWMDIV_2);

	//The PWM peripheral must be enabled for use.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

	//Configure the GPIO pin muxing to select PWM0 functions for these pins.
	//This step selects which alternate function is available for these pins.
	GPIOPinConfigure(GPIO_PB6_M0PWM0);
	GPIOPinConfigure(GPIO_PB7_M0PWM1);

	//Configure the PWM function for these pins.
	GPIOPinTypePWM(GPIO_PORTB_BASE, (GPIO_PIN_6 | GPIO_PIN_7));

	//Configure the PWM generator for count down mode with immediate updates
	//to the parameters.
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

	//Set the period. For a 300Hz frequency, remember the PWM clock is the
	//system clock divided (see code above)
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, (SysCtlClockGet() / 2 / PWM_FREQUENCY));

	//Set the base pulse width of PWM0 (zero motor output).
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, PWM_BASE_PULSE);

	//Set the base pulse width of PWM0 (zero motor output).
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PWM_BASE_PULSE);

	//Start the timers in generator 0.
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);

	//Enable the outputs.
	PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT | PWM_OUT_1_BIT), true);
}
void motors_setpwm(float left, float right)
{
	if(left > PWM_HARDLIMIT) left = PWM_HARDLIMIT;
	if(left < -PWM_HARDLIMIT) left = -PWM_HARDLIMIT;
	if(right > PWM_HARDLIMIT) right = PWM_HARDLIMIT;
	if(right < -PWM_HARDLIMIT) right = -PWM_HARDLIMIT;
	
	PWMPulseWidthSet(PWM0_BASE, 
	                 PWM_OUT_0, 
	                 (PWM_BASE_PULSE + left*PWM_PULSE_DIFF*left_motor.polarity));
	PWMPulseWidthSet(PWM0_BASE, 
	                 PWM_OUT_1, 
	                 (PWM_BASE_PULSE + right*PWM_PULSE_DIFF*right_motor.polarity));
}
