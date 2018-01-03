#ifndef I2CMAN_H
#define I2CMAN_H

#include <stdint.h>
#include <stdbool.h>

#pragma anon_unions

typedef struct
{
	enum {
		OP_SINGLE_SEND,
		OP_BURST_SEND_START,
		OP_BURST_SEND_CONT,
		OP_BURST_SEND_FINISH,
		OP_SINGLE_RECEIVE,
		OP_BURST_RECEIVE_START,
		OP_BURST_RECEIVE_CONT,
		OP_BURST_RECEIVE_FINISH,
	} op;
	uint8_t address;
	union
	{
		uint8_t value;
		uint8_t * valueptr;
	};
} i2c_op_t;

#define I2C_NOACK 0x00
#define I2C_ARBLOST 0x01

#define I2CMAN_INTERRUPT INT_I2C0
extern void i2cman_handler(void);

extern void i2cman_init(void);
extern void i2cman_abort(void);
extern bool i2cman_execute(const i2c_op_t * _begin, const i2c_op_t * _end, void (* succ)(void), void (* err)(uint32_t));

#endif
