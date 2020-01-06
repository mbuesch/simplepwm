#ifndef UTIL_H_
#define UTIL_H_

#define abs(x)			((x) >= 0 ? (x) : -(x))
#define min(a, b)		((a) < (b) ? (a) : (b))
#define max(a, b)		((a) > (b) ? (a) : (b))
#define ARRAY_SIZE(a)		(sizeof(a) / sizeof((a)[0]))
#define memory_barrier()	__asm__ __volatile__("" : : : "memory")

#endif /* UTIL_H_ */
