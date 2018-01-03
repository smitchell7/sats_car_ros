#ifndef IOBUFFER_H
#define IOBUFFER_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

// Implements a generic byte FIFO

typedef struct fifo fifo_t;

struct fifo {
	uint8_t * data;
	size_t size;
	int begin;
	int end;
};

#define FIFO_INITIALIZER(D, SZ) { \
	.data = D, \
	.size = SZ, \
	.begin = 0, \
	.end = 0 }

// Clears the given fifo
void fifo_clear(fifo_t * fifo);

// Enqueues a single byte onto the fifo.
//
// Returns whether the operation was successful.
bool fifo_putbyte(fifo_t * fifo, uint8_t byte);
// Enqueues a list of bytes onto the fifo. Only
// enqueues the data if the entire list will fit.
//
// Returns whether the operation was successful.
bool fifo_putbytes(fifo_t * fifo, const uint8_t * data, unsigned int datasize);
// Retrieves a byte from the fifo.
// Called by the ISR when a byte is ready for transmission.
//
// Returns whether the operation was successful.
bool fifo_getbyte(uint8_t * b, fifo_t * fifo);


#endif
