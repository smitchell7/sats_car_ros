
#include <stdint.h>
#include <string.h>

#include "fifo.h"

void fifo_clear(fifo_t * fifo)
{
	fifo->begin = 0;
	fifo->end = 0;
}

bool fifo_putbyte(fifo_t * fifo, uint8_t byte)
{
	unsigned int next_end = (fifo->end + 1) % fifo->size;
	if(next_end == fifo->begin)
		return false; // there might be one byte left, but 
					  // begin == end means the buffer is empty

	fifo->data[fifo->end] = byte;
	fifo->end = next_end;

	return true;
}
bool fifo_putbytes(fifo_t * fifo, const uint8_t * data, unsigned int datasize)
{
	unsigned int end = fifo->end;
	for(unsigned int i = 0 ; i < datasize ; i ++)
	{
		fifo->data[end] = data[i];
		
		end = (end + 1) % fifo->size;
		if(end == fifo->begin)
			return false; // not enough room eh
	}
	
	fifo->end = end; // actually apply the change, since we made it through
	return true;
}
bool fifo_getbyte(uint8_t * b, fifo_t * fifo)
{
	if(fifo->begin != fifo->end)
	{
		*b = fifo->data[fifo->begin];
		fifo->begin = (fifo->begin + 1) % fifo->size;
		return true;
	}
	
	return false;
}
