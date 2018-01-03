#ifndef __LOGGER__H__
#define __LOGGER__H__

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
//!brief Representation of a quadrature encoder
typedef struct
{
	float time; 
	float data[4];
} Data_Packet;
//extern Data_Packet get_data(void);
extern int get_data(Data_Packet * outData); 

extern void reset_data_idx(void);
extern int store_data(float time, float * datas);

#endif
