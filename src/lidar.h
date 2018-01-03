#ifndef LIDAR_H
#define LIDAR_H

typedef struct
{
	float distance;
	float velocity;
	float acceleration;
} lidar_module_t;

// Global variables are named with underscores
// at the beginning and end.
extern lidar_module_t _frontlidar_;

// lidar_tryread() should be called every LIDAR_SAMPLE_PERIOD seconds,
// its internal velocity filter is tuned for this
#define LIDAR_SAMPLE_TIME 0.025

extern void lidar_tryread(void);
extern void lidar_init(void);

#endif
