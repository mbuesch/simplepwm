#ifndef UTIL_H_
#define UTIL_H_

#include "compat.h"

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <string.h>


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

/* Limit an unsigned integer to uint8_t/uint16_t range. */
#define lim_u8(v)	((uint8_t)clamp((v), (__typeof__(v))0, (__typeof__(v))UINT8_MAX))
#define lim_u16(v)	((uint16_t)clamp((v), (__typeof__(v))0, (__typeof__(v))UINT16_MAX))

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

/* Do-not-inline function attribute. */
#define noinline		__attribute__((__noinline__))

/* Always-inline function attribute. */
#define alwaysinline		inline __attribute__((__always_inline__))

/* Packed structure. */
#define _packed			__attribute__((__packed__))

/* Build-time assertion.
 * 'cond' must be a compile-time constant.
 * Build will fail, if 'cond' is false.
 */
#define build_assert(cond)	((void)sizeof(char[1 - 2 * !(cond)]))

/* Code flow attributes */
#define noreturn		__attribute__((__noreturn__))
#define _mainfunc		__attribute__((__OS_main__))
#if defined(__GNUC__) && __GNUC__ >= 4 && __GNUC_MINOR__ >= 5
# define unreachable()		__builtin_unreachable()
#else
# define unreachable()		while (1)
#endif

/* Code section attributes */
#define section_init3		__attribute__((naked, used, section(".init3")))

/* Data section attributes */
#define section_noinit		__attribute__((section(".noinit")))

/* Non-standard integer types. */
typedef __int24		int24_t;
#define int24_t int24_t
typedef __uint24	uint24_t;
#define uint24_t uint24_t


/* Disable interrupts globally. */
static alwaysinline void irq_disable(void)
{
	cli();
	memory_barrier();
}

/* Enable interrupts globally. */
static alwaysinline void irq_enable(void)
{
	memory_barrier();
	sei();
}

/* Save flags and disable interrupts globally. */
static alwaysinline uint8_t irq_disable_save(void)
{
	uint8_t sreg = SREG;
	irq_disable();
	return sreg;
}

/* Restore interrupt flags. */
static alwaysinline void irq_restore(uint8_t sreg_flags)
{
	memory_barrier();
	SREG = sreg_flags;
}

typedef uint16_t le16_t;

/* Convert native endianness to little-endian 16-bit. */
static inline le16_t to_le16(uint16_t x)
{
	/* Architecture is little-endian. */
	return (le16_t)x;
}

/* Convert little-endian 16-bit to native endianness. */
static inline uint16_t from_le16(le16_t x)
{
	/* Architecture is little-endian. */
	return (uint16_t)x;
}

/* Reboot into the bootloader. */
static inline noreturn void enter_bootloader(void)
{
	/* Use the watchdog timer to reboot the system. */
	irq_disable();
	wdt_enable(WDTO_15MS);
	while (1);
	unreachable();
}

/* Port toggle for debugging purposes. */
#define debug_toggle(portname, portbit, delay_us, count)			\
	do {									\
		uint8_t _dt_i;							\
		DDR##portname |= 1u << (portbit);				\
		for (_dt_i = 0; _dt_i < (uint8_t)(count); _dt_i++) {		\
			PIN##portname |= 1u << (portbit);			\
			_delay_us(delay_us);					\
		}								\
	} while (0)

#endif /* UTIL_H_ */
