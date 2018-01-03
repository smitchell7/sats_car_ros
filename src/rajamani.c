
#include "rajamani.h"
#include "math.h"
#include "timing.h"

// coefficients / configuration
float kp = 0, kd = 0;
float alpha = 0, beta1 = 0, beta2 = 0;
float h = 0; //Recommended value of 0.5, but varies in different algorithms. 
//algorithm 2 and 3:
float kh = .1;
float desired_vel = 0.0;
float inter_veh_dist = 0.0; //Because the yanakiev_csp controller needs a larger value that the const INTER_VEH_DISTANCE.
//algorithm 4:
float tau = 0, K1 = 0, K5 = 0;
//algorithm 5:
float Cp = 0, Cv = 0, Ca = 0, Kv = 0, Ka = 0;

// memory variables
static float v_old = 0;
static float fb_fout = 0;
static float v_ref = 0;

extern unsigned int ll_ticks;

const float INTER_VEH_DISTANCE = 0.5;

// state variables
float accel = 0;

void yanakiev_ctg(float lidar_dist, float lidar_vel, float vel, float time)
{
	accel = (kp*(lidar_dist - vel * h - INTER_VEH_DISTANCE) + kd*lidar_vel); // simple PD controller
	//accel = (kp*(lidar_dist - vel * 0.5 - INTER_VEH_DISTANCE) + kd*lidar_vel) + ki*sin(1.4*time); // simple PD controller
}
void low_level_reset(float a,float b1,float b2)
{
	alpha = a;
	beta1 = b1;
	beta2 = b2;
}

float rajamani_lowlevel(float velocity)
{
	float v = velocity;
	// this function returns the desired voltage to achieve that acceleration
	return 1.0/alpha*(accel + beta1*v + beta2*(v*v) );
}
void velocity_highlevel(float lidar_dist, float lidar_vel, float vel)
{
	accel = kd*(desired_vel - vel); 
	/*
	if((lidar_dist < 2*INTER_VEH_DISTANCE + h*desired_vel) && (lidar_dist >0.03))
		accel = kp*(lidar_dist - 4*INTER_VEH_DISTANCE - h*vel); 
	*/
}
void rajamani_reset(float _kp, float _h, float _kd)
{
	kp = _kp;
	h = _h;
	kd = _kd;

	// reset some memory variables everytime
	v_old = 0;
	fb_fout = 0;
	v_ref = 0;
}

void yanakiev_vtg(float lidar_dist, float lidar_vel, float vel, float time)
{
	accel = (kp * (lidar_dist - INTER_VEH_DISTANCE ) + (kd + kp * kh * desired_vel) * (lidar_vel + vel)- (kd + kp * h + kp * kh * desired_vel) * vel); //kp * h0 * vel);
	// x'' = kp * (x_i+1 - x_i) + (kd + kp*kh*vd) * (x'_i+1 - x'_i) - kp*h0*x'_i
}

extern void jonathan_reset(float _kp, float _kh, float _kd, float desVel, float _h )
{
	kp = _kp;
	kh = _kh;
	kd = _kd;
	desired_vel = desVel;
	h = _h;
	
	//reset some memory variables everytime
	//why? because that's what Rajamani did
	v_old = 0;
	fb_fout = 0;
	v_ref = 0;
	accel = 0; 
}
void csp_reset(float _kp, float _kh, float _kd, float desVel, float _h, float _inter_veh_distance )
{
	kp = _kp;
	kh = _kh;
	kd = _kd;
	desired_vel = desVel;
	h = _h;
	inter_veh_dist = _inter_veh_distance;
	
	//reset some memory variables everytime
	//why? because that's what Rajamani did
	v_old = 0;
	fb_fout = 0;
	v_ref = 0;
	accel = 0; 
}
void yanakiev_csp(float lidar_dist, float lidar_vel, float vel, float time)
{
	accel = kp * (lidar_dist - inter_veh_dist) + kd * lidar_vel + kh * (desired_vel - vel);
}
void rajamani_ctg(float lidar_dist, float lidar_vel, float lidar_accel, float vel, float qei_accel, float time)
{
	float delta = lidar_dist - INTER_VEH_DISTANCE;							//relative distance; pos_err_vic
	float delta_dot = lidar_vel;																//velocity error; vel_err_vic
	float jerk = (1.0/tau) * (-lidar_accel * K1 + qei_accel * (K1*K5*h - 1) - (1.0/h) * (1 - K1*K5*h) * delta_dot - (K5/h) * delta - K5*vel);
	accel += jerk * HIGH_LEVEL_TIME;
}

// JERK CONTROLLER
void rajamani_ctg_reset(float _tau, float _k1, float _k5, float _h)
{
	tau = _tau;
	K1 = _k1;
	K5 = _k5;
	h = _h;
	
	//reset some memory variables everytime
	//why? because that's what Rajamani did
	v_old = 0;
	fb_fout = 0;
	v_ref = 0;
	accel = 0; 
}
// JERK CONTROLLER
void chien_ctg(float lidar_dist, float lidar_vel, float lidar_accel, float vel, float qei_accel, float time)
{
	float delta = lidar_dist - INTER_VEH_DISTANCE;
	float delta_dot = lidar_vel;
	float jerk = (1.0 / (1.0 + h * Ca)) * (Cp * delta + Cv * delta_dot + Ca * lidar_accel + Kv * vel + Ka * qei_accel);
	accel += jerk * HIGH_LEVEL_TIME;
}

void chien_ctg_reset(float _h, float _Ca, float _Cp, float _Cv, float _Kv, float _Ka)
{
	h = _h;
	Ca = _Ca;
	Cp = _Cp;
	Cv = _Cv;
	Kv = _Kv;
	Ka = _Ka;
	
	//reset some memory variables everytime
	//why? because that's what Rajamani did
	v_old = 0;
	fb_fout = 0;
	v_ref = 0;
	accel = 0; 
}

float LL_controller_mrac(float v,
	float g1, float g2, float g3, float gl1, float gl2, float gl3,
	float Ts)
{
	// this function uses a direct model reference adaptive control
	// (direct-MRAC) approach to achieve the desired acceleration

	// r is desired acceleration, v is current estimate (or measure)
	// of vehicle's own velocity, Ts is the sampling time at which the
	// low level controller/quadrature encoder works.

	// These values have been tested in MATLAB:
	// g1 = 5, g2 = 2, g3 = 3;
	// gl1 = 30, gl2 = 30, gl3 = 30

	// adataption variables
	static float x11 = 0;
	static float x21 = 0;
	static float x31 = 0;
 
	static float x12 = 0;
	static float x22 = 0;
	static float x32 = 0;
 
	static float k1 = 0;
	static float k2 = 0;
	static float k3 = 0;
 
	// reference model variables
	static float r_0 = 0;
	static float v_des = 0;
	float e;
	
	float r = accel;

	float u = k1*v + k2*r + k2*( (v > 0) - (v < 0) );

	// reference model
	v_des = v_des + Ts/2*(r + r_0);
	r_0 = r;

	// adaptation
	e = v - v_des;

	k1 = (x11 + x12)*Ts/2;
	k2 = (x21 + x22)*Ts/2;
	k3 = (x31 + x32)*Ts/2;

	x12 = x11;
	x22 = x21;
	x32 = x31;

	x11 = x11 + ( g1*e*v - gl1*k1 );
	x21 = x21 + ( g2*e*r - gl2*k2 );
	x31 = x31 + ( g3*e*( (v > 0) - (v < 0) ) - gl3*k3 );

	if (k1 > 5) k1 = 5;
	if (k1 < -5) k1 = -5;
	if (k3 > 5)	k3 = 5;
	if (k3 < -5) k3 = -5;
	if (k2 > 5)	k2 = 5;
	if (k2 < -5) k2 = -5;

	// controller output
	return u;
	//return k1*v + k2*r ;
	//return 0.3;
	// this is pwm output from -1 to 1.

}

float ll_controller_af(float v, float V, float Ts)
{
	// these can be thought of as states of the system
	static float v_old = 0;
	static float fb_fout = 0;
	static float ff_fout = 0;

	// just parameters for the lpfs
	float g1 = 0.25;
	float g2 = 0.25;

	float u;
	
	// output can be set at end with 'next' value or at start, with
	// current values. start should be preferred.

	// if you want to bypass the feedforward lpf, use second output
	// assignment


	// in one iteration, all of this happens, so every step
	// should be independent of any previous step that sets a static

	// assign output 
	u = 1/alpha/V*( accel + beta1*v);
	//u = 1/alpha/V*( (accel + fb_fout) + beta1*v);

	// feedforward lpf
	ff_fout = (1-g2)*ff_fout + g2*(accel + fb_fout);

	// feedback lpf
	fb_fout = (1-g1)*fb_fout + g1*(accel + fb_fout - (v - v_old)/Ts);

	// set old velocity
	v_old = v;

	return u;
}

float ll_controller_avf(float v, float V, float Ts)
{
	// from simulations, Gv should be close to 4

	// these can be thought of as states of the system
	//static float v_old = 0;
	//static float fb_fout = 0;
	//static float v_ref = 0;
	// now set outside, globally
	
	// just parameters for the lpfs
	float g = 0.02;
	float Gv = 5;

	float u;
	
	// output can be set at end with 'next' value or at start, with
	// current values. start should be preferred.

	// in one iteration, all of this happens, so every step
	// should be independent of any previous step that sets a static

	// assign output 
	u = 1/alpha/V*( (accel + fb_fout) + beta1*v);

	// feedback lpf
	fb_fout = (1-g)*fb_fout + g*(accel + fb_fout - (v - v_old)/Ts + 
		Gv*(v_ref - v) );

	// set old velocity
	v_old = v;

	// reference model
	v_ref = v_ref + Ts*accel;
	v_ref = (v_ref > 9) ? 9 : v_ref;
	v_ref = (v_ref < 0) ? 0 : v_ref;

	return u;
}
