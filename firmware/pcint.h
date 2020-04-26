#ifndef PCINT_H_
#define PCINT_H_

#include "main.h"


#if IS_ATMEGAx8
# define USE_PCINT	1
# define IF_PCINT(...)	__VA_ARGS__
#else
# define USE_PCINT	0
# define IF_PCINT(...)	/* nothing */
#endif


typedef void (*pcint_callback_t)(void);

void pcint_enable(uint8_t index, bool enable);
void pcint_register_callback(uint8_t index, pcint_callback_t cb);

#endif /* PCINT_H_ */
