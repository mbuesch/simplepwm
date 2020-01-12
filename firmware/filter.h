#ifndef FILTER_H_
#define FILTER_H_

#include "util.h"


struct lp_filter {
	uint32_t filter_buf;
};


static inline void lp_filter_set(struct lp_filter *lp,
				 uint16_t new_value)
{
	lp->filter_buf = new_value;
}

static inline void lp_filter_reset(struct lp_filter *lp)
{
	lp_filter_set(lp, 0u);
}

uint16_t lp_filter_run(struct lp_filter *lp,
		       uint8_t shift,
		       uint16_t in);

#endif /* FILTER_H_ */
