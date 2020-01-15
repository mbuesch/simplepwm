#ifndef UTIL_H_
#define UTIL_H_

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>


/* Return the smaller value of 'a' and 'b'. */
#define min(a, b)	({						\
		__typeof__(a) __amin = (a);				\
		__typeof__(b) __bmin = (b);				\
		(__typeof__(a))(__amin < __bmin ? __amin : __bmin);	\
	})

/* Return the bigger value of 'a' and 'b'. */
#define max(a, b)	({						\
		__typeof__(a) __amax = (a);				\
		__typeof__(b) __bmax = (b);				\
		(__typeof__(a))(__amax > __bmax ? __amax : __bmax);	\
	})

/* Return 'value' clamped inbetween 'min_val' and 'max_val'. */
#define clamp(value, min_val, max_val)		\
	max(min(value, max_val), min_val)

/* Return the absolute value of 'val' */
#undef abs
#define abs(val)	({			\
		__typeof__(val) __val = (val);	\
		__val >= 0 ? __val : -__val;	\
	})


/* Get the number of elements in a C array. */
#define ARRAY_SIZE(a)		(sizeof(a) / sizeof((a)[0]))

/* Memory barrier. */
#define memory_barrier()	__asm__ __volatile__("" : : : "memory")


#endif /* UTIL_H_ */
