#ifndef CURVE_H_
#define CURVE_H_

struct curve_point {
	uint16_t x;
	uint16_t y;
};

#define CURVE_POINT(_x, _y)	{ .x = (uint16_t)(_x), \
				  .y = (uint16_t)(_y), }

uint16_t curve_interpolate(const struct curve_point __flash *curve,
			   uint8_t curve_size,
			   uint16_t x);

#endif /* CURVE_H_ */
