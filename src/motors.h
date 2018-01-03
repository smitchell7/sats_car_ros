#ifndef MOTORS_H
#define MOTORS_H

#define MOTOR_NORMAL  0
#define MOTOR_REVERSE 1

extern void motors_init(unsigned int polarity_l, unsigned int polarity_r);
extern void motors_setpwm(float left, float right);

#endif
