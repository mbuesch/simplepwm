#ifndef COMPAT_H_
#define COMPAT_H_

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/cpufunc.h>
#include <util/delay.h>


#ifndef TIFR
# define TIFR	TIFR0
#endif
#ifndef TIMSK
# define TIMSK	TIMSK0
#endif
#ifndef WDIE
# define WDIE	WDTIE
#endif

#endif /* COMPAT_H_ */
