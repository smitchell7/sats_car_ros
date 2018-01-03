
#include "FIRFilter.h"

// returns derivative, keeps a history for 4th order approximation
float FIRFilter_flush(FIRFilter * filter, float x)
{
	/*
	unsigned int size = filter->size;
	float * filterdata = filter->filter;
	float * xbuf = filter->xbuf;
	
	float sum = x * filterdata[0];
	
	int fidx = 1;
	int xidx = filter->idx;
	while(true)
	{
		sum += filterdata[fidx] * xbuf[xidx];
		
		--xidx;
		++fidx;
		if(xidx < 0)
			xidx = size - 2;
		if(fidx >= size)
			break;
	}
	
	filter->idx ++;
	filter->xbuf[idx] = x;
	if(filter->idx >= size)
		filter->idx = 0;
	
	return fx;
	*/
	return 0;
}
