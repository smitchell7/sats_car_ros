
#include "lidar.h"
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "i2cman.h"

i2c_op_t request_ops[] = {
	// lidar[0x00] <= 0x04
	{ .op = OP_BURST_SEND_START,     .address = 0x62, .value = 0x00 },
	{ .op = OP_BURST_SEND_FINISH,    .address = 0x62, .value = 0x04 },
};

uint8_t dist_low = 0;
uint8_t dist_high = 0;
i2c_op_t read_dist_ops[] = {
	// lidar[0x0f, ...] => dist_high, dist_low
	{ .op = OP_SINGLE_SEND,          .address = 0x62, .value = 0x8f },
	{ .op = OP_BURST_RECEIVE_START,  .address = 0x62, .valueptr = &dist_high },
	{ .op = OP_BURST_RECEIVE_FINISH, .address = 0x62, .valueptr = &dist_low }
};

lidar_module_t _frontlidar_ = {0};
/*
My magic filter
static float vfilt[6] = {
	8.631589294816258,
	-1.308161808956109,
	0.776690541983876,
	-0.776690541983876,
	1.308161808956109,
	-8.631589294816258
};
*/

//we have a history of size 20, but 32 is a power of two ;)
#define DELAYLESS_DIFF_SIZE 32
#define DELAYLESS_DIFF_MASK 0x1F

typedef struct {
  double dbuf[DELAYLESS_DIFF_SIZE];
  unsigned int dbufi;
  double q[3];
} delayless_diff_t;

#define DELAYLESS_DIFF_INIT { {0}, 0, {0} }

#define N 20

static double DELAYLESS_DIFF_C[] = { 0.1057169439, -0.02719298246, 0.002335383914 };
static double DELAYLESS_DIFF_R[] = {-1, -(double)N, -.5*((double)N)*((double)(N+1))};

// This function describe a valid history of x for N in [-DBUF_SIZE, -1]
// undefined elsewhere
static double dbuf_get(delayless_diff_t * st, int n)
{
	return st->dbuf[(n + DELAYLESS_DIFF_SIZE + st->dbufi) & DELAYLESS_DIFF_MASK];
}
// Adds an element to the x history
static double delayless_diff_in(delayless_diff_t * st, double x)
{
	double xdelayed = dbuf_get(st, -N);
	
	st->q[0] += DELAYLESS_DIFF_R[0]*xdelayed + x;
	st->q[1] += DELAYLESS_DIFF_R[1]*xdelayed + st->q[0];
	st->q[2] += DELAYLESS_DIFF_R[2]*xdelayed + st->q[1];
	float dx = DELAYLESS_DIFF_C[0]*st->q[0] + 
             DELAYLESS_DIFF_C[1]*st->q[1] + 
             DELAYLESS_DIFF_C[2]*st->q[2];
  
	st->dbuf[st->dbufi & DELAYLESS_DIFF_MASK] = x;
	st->dbufi = (st->dbufi+1) & DELAYLESS_DIFF_MASK;
  
  return dx;
}

delayless_diff_t position_velocity_diff = DELAYLESS_DIFF_INIT;
delayless_diff_t velocity_acceleration_diff = DELAYLESS_DIFF_INIT;

bool request_sent = false;

void read_succ(void);
void read_erro(uint32_t err);
void request_succ(void)
{
	request_sent = true;
	// i2cman_execute(read_dist_ops, read_dist_ops + 3, read_succ, read_erro);
}
void request_erro(uint32_t err)
{
	if(err == I2C_ARBLOST)
		i2cman_init();
	i2cman_abort();
	
	request_sent = false;
	// meh, don't care
}
float lastx = 0;
void read_succ(void)
{
	//noice lets do this
	double x;
	double 	old_vel = _frontlidar_.velocity; 

	if(dist_low == 0 && dist_high == 0)
	{
		// just keep the last value
		x = lastx;
	}
	else
	{
		x = (double)(dist_low | ((unsigned int)dist_high << 8))/100.0;
		
		if(x > 40.0)
			// just keep the last value
			x = lastx;
	}
  lastx = x;
  
  double dx = delayless_diff_in(&position_velocity_diff, x);
  double ddx = delayless_diff_in(&velocity_acceleration_diff, dx);
	
	_frontlidar_.distance = x;
	_frontlidar_.velocity = dx/LIDAR_SAMPLE_TIME;
	_frontlidar_.acceleration = ddx/LIDAR_SAMPLE_TIME;
	
	request_sent = false;
	// yay!  request immediately
	i2cman_execute(request_ops, request_ops + 2, request_succ, request_erro);
}
void read_erro(uint32_t err)
{
	if(err == I2C_ARBLOST)
		i2cman_init();
	i2cman_abort();
	
	// Now we'll resend a request!
	request_sent = false;
}

void lidar_tryread()
{
	// error_num = 0;
	if(request_sent)
		i2cman_execute(read_dist_ops, read_dist_ops + 3, read_succ, read_erro);
	else
		i2cman_execute(request_ops, request_ops + 2, request_succ, request_erro);
}
void lidar_init()
{
  position_velocity_diff = (delayless_diff_t)DELAYLESS_DIFF_INIT;
  velocity_acceleration_diff = (delayless_diff_t)DELAYLESS_DIFF_INIT;
	_frontlidar_.distance = 0;
	_frontlidar_.velocity = 0;
	_frontlidar_.acceleration = 0;
	i2cman_init();
}
