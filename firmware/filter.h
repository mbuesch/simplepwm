#ifndef FILTER_H_
#define FILTER_H_

#include "util.h"


struct lp_filter {
	uint32_t filter_buf;
	bool initialized;
};


uint16_t lp_filter_run(struct lp_filter *lp,
		       uint8_t shift,
		       uint16_t in);

void lp_filter_reset(struct lp_filter *lp);

#endif /* FILTER_H_ */
