#ifndef DEBUG_H_
#define DEBUG_H_

#include "main.h"
#include "util.h"


#if DEBUG && IS_ATMEGAx8


void dfprintf(const char __flash *fmt, ...);
#define dprintf(fmt, ...)	dfprintf(PSTR(fmt) ,##__VA_ARGS__)
void debug_prepare_deep_sleep(void);
void debug_init(void);


#else /* DEBUG && IS_ATMEGAx8 */


#define dprintf(fmt, ...)	do {} while (0)
static inline void debug_prepare_deep_sleep(void) {}
static inline void debug_init(void) {}


#endif /* DEBUG && IS_ATMEGAx8 */
#endif /* DEBUG_H_ */
