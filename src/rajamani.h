#ifndef RAJAMANI_H
#define RAJAMANI_H

// Does some processing
extern void yanakiev_ctg(float lidar_dist, float lidar_vel, float vel, float time);
// Returns a voltage to be applied to the motors
extern float rajamani_lowlevel(float vel);
// Reset and reinitialize
extern void rajamani_reset(float kp, float ki, float kd);
// Returns a voltage to be applied to the motors
extern void yanakiev_vtg(float lidar_dist, float lidar_vel, float vel, float time);
extern void velocity_highlevel(float lidar_dist, float lidar_vel, float vel); 
extern void low_level_reset(float a,float b1,float b2); 
extern void yanakiev_csp(float lidar_dist, float lidar_vel, float vel, float time); 
extern void rajamani_ctg(float lidar_dist, float lidar_vel, float lidar_accel, float vel, float qei_accel, float time);
extern void rajamani_ctg_reset(float _tau, float _k1, float _k5, float _h);
extern void chien_ctg(float lidar_dist, float lidar_vel, float lidar_accel, float vel, float qei_accel, float time);
extern void chien_ctg_reset(float _h, float _Ca, float _Cp, float _Cv, float _Kv, float _Ka);
extern void csp_reset(float kp, float ki, float kd, float desVel, float h, float inter_veh_dist);

extern void jonathan_reset(float kp, float ki, float kd, float desVel, float h);
extern float LL_controller_mrac(float v,
	float g1, float g2, float g3, float gl1, float gl2, float gl3,
	float Ts);
	
extern float ll_controller_af(float v, float V, float Ts);
extern float ll_controller_avf(float v, float V, float Ts);
#endif
