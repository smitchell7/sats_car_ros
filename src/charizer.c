
#include "charizer.h"
#include "lidar.h"
#include "qei.h"
#include <math.h>
#include <stdbool.h>

float _charizer_pwm_ = 0;

static float velocity[400];
static unsigned int velocity_pos = 400;

static float T = 0;

void charizer_velocity(float v)
{
	if(velocity_pos >= 400)
		return;
	
	velocity[velocity_pos] = v;
	velocity_pos ++;
}
static void test(float pwm)
{
	// Grab first position
	float x0 = (_qei_leftmotor_.distance + _qei_rightmotor_.distance)/2.0f;
	
	bool invalid = true;
	while(invalid)
	{
		invalid = false;
		// Go!
		_charizer_pwm_ = pwm;
		velocity_pos = 0;
		// While we're still receiving data
		while(velocity_pos < 400)
		{
			// If something's in the way...
			if(_frontlidar_.distance < 1.0)
			{
				// Stop everything
				_charizer_pwm_ = 0;
				velocity_pos = 0;
				invalid = true;
				break;
			}
		}
		
		// Back it up
		_charizer_pwm_ = -.2;
		while((_qei_leftmotor_.distance + _qei_rightmotor_.distance)/2.0f > x0);
	
		// Stop
		_charizer_pwm_ = 0;
	}
}
static float find_vfinal()
{
	// Average of last 25% of velocity samples
	float sum = 0;
	for(int i = 300 ; i < 400 ; i ++)
	{
		sum += velocity[i];
	}
	return (sum / 100);
}
static float find_kv(float P)
{
	return P / find_vfinal();
}
static float find_ka(float P, float kv, float h)
{
	// Least squares exponential curve fitting, solves directly
	// for ka
	float num = 0, den = 0;
	for(int i = 0 ; i < 200 ; i ++)
	{
		num += kv*h*i * kv*h*i;
		den += log2(1 - kv*velocity[i]/P)*(-kv*h*i);
	}
	
	return num / den;
}

void charizer_go(float * db_out)
{
	// Perform a binary search for the deadband cutoff
	float del_db = .05;
	float db = .1;
	
	while(del_db > .0001)
	{
		test(db);
		if(find_vfinal() > .01)
			db -= del_db;
		else
			db += del_db;
		del_db /= 2;
	}

	*db_out = db;
	
	// Accelerate with power 0.2, and assess the results
	test(db + 0.2*(1-db));
	float kv = find_kv(0.2);
	float ka = find_ka(0.2, kv, T);
}
void charizer_init(float _T)
{
	// not much to do here, m'lady
	_charizer_pwm_ = 0;
	T = _T;
}
