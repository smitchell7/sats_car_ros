#ifndef FIRFilter_H
#define FIRFilter_H

typedef struct
{
	unsigned int size;
	unsigned int idx;
	float * xbuf;
	float * filter;
} FIRFilter;

#define FIRFILTER_INIT(_filt_, _buf_, _size_) { \
	.size = _size_, \
	.idx = 0, \
	.xbuf = _buf_, \
	.filter = _filt_ \
}

// this is broken don't use it
extern float FIRFilter_flush(FIRFilter * filter, float x);

#endif
