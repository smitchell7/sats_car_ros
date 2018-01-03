#include "logger.h"

const size_t BUFFER_SIZE = 4*90; 

Data_Packet data_log[BUFFER_SIZE]; 
int ind = 0; 

int store_data(float time, float * datas)
{
	data_log[ind].time = time; 
	memcpy(data_log[ind].data,datas,20); 
	
	int oldind = ind; 
	ind = (ind+1)%BUFFER_SIZE; 
	return oldind; 
}
int get_data(Data_Packet * outData)
{
	outData[0] = data_log[ind]; // may need to use memcpy here
	
	int oldind = ind; 
	ind = (ind+1)%BUFFER_SIZE; 
	return oldind; 
}
void reset_data_idx(void)
{
	ind = 0; 
}
