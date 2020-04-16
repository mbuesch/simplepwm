#ifndef COLOR_H_
#define COLOR_H_

#include "util.h"


void hsl2rgb(uint16_t *r, uint16_t *g, uint16_t *b,
	     uint16_t h, uint16_t s, uint16_t l);


#endif /* COLOR_H_ */
