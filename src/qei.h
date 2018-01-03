#ifndef QEI_H
#define QEI_H

//!brief Representation of a quadrature encoder
typedef struct
{
	float distance;
	float velocity;
	float acceleration;
} qei_t;

// Global variables are named with underscores
// at the beginning and end.
extern qei_t _qei_leftmotor_;
extern qei_t _qei_rightmotor_;

#define QEI_LEFT_INTERRUPT INT_QEI1
#define QEI_RIGHT_INTERRUPT INT_QEI0
extern void qei_left_handler(void);
extern void qei_right_handler(void);
extern void qei_recalibrate(float gear_ratio, float wheel_circumference);

// \brief Initialize QEI modules
//
// \description
// Sets up QEI0 and QEI1 for velocity (delta position) measurements
// at time intervals of 25ms, and configures based on gear ratio
// and wheel circumference.
//
// `_qei_leftmotor_` and `_qei_rightmotor_` will then be updated via interrupt.
//
// \note Defines QEI0_Handler and QEI1_Handler
extern void qei_init(float gear_ratio, float wheel_circumference);

#endif
