#ifndef CHARIZER_H
#define CHARIZER_H

extern float _charizer_pwm_;

extern void charizer_velocity(float v);

extern void charizer_go(float * db_out);
extern void charizer_init(float T);

#endif
