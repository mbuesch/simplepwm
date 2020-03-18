#ifndef MOVINGAVG_H_
#define MOVINGAVG_H_

#include "util.h"


typedef uint16_t movingavg_t;
typedef __uint24 movingavgsum_t;

struct movingavg {
	uint8_t size;
	uint8_t count;
	uint8_t begin;
	uint8_t end;
	movingavgsum_t avgsum;
	movingavg_t buf[0];
};

#define DEFINE_MOVINGAVG(name, maxsize)		\
	struct {				\
		struct movingavg m;		\
		movingavg_t buf[maxsize];	\
	} name

movingavg_t _movingavg_calc(struct movingavg *m, movingavg_t newvalue);
#define movingavg_calc(_m, _newvalue)	_movingavg_calc(&((_m)->m), (_newvalue))

void _movingavg_init(struct movingavg *m, uint8_t size);
#define movingavg_init(_m, _size)	_movingavg_init(&((_m)->m), (_size))

#endif /* MOVINGAVG_H_ */
