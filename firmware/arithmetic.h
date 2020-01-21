#ifndef ARITHMETIC_H_
#define ARITHMETIC_H_

#include "util.h"


#define ARITHMETIC_ASM_AVR8	1


/* Saturated unsigned addition (32 bit). */
static inline uint32_t add_sat_u32(uint32_t a, uint32_t b)
{
#if ARITHMETIC_ASM_AVR8
	__asm__ __volatile__(
		"add %A0, %A1 \n"
		"adc %B0, %B1 \n"
		"adc %C0, %C1 \n"
		"adc %D0, %D1 \n"
		"brcc 1f \n"
		"ldi %A0, 0xFF \n"
		"ldi %B0, 0xFF \n"
		"movw %C0, %A0 \n"
/*		"ldi %C0, 0xFF \n"
		"ldi %D0, 0xFF \n" */
		"1: \n"
		: "+d" (a)
		: "d" (b)
	);
	return a;
#else
	uint32_t x = (uint32_t)(a + b);
	if (x < a)
		return UINT32_MAX;
	return x;
#endif
}

#ifdef uint24_t
/* Saturated unsigned addition (24 bit). */
static inline uint24_t add_sat_u24(uint24_t a, uint24_t b)
{
#if ARITHMETIC_ASM_AVR8
	__asm__ __volatile__(
		"add %A0, %A1 \n"
		"adc %B0, %B1 \n"
		"adc %C0, %C1 \n"
		"brcc 1f \n"
		"ldi %A0, 0xFF \n"
		"ldi %B0, 0xFF \n"
		"ldi %C0, 0xFF \n"
		"1: \n"
		: "+d" (a)
		: "d" (b)
	);
	return a;
#else
	uint24_t x = (uint24_t)(a + b);
	if (x < a)
		return (uint24_t)0xFFFFFFlu;
	return x;
#endif
}
#endif /* uint24_t */

/* Saturated unsigned addition (16 bit). */
static inline uint16_t add_sat_u16(uint16_t a, uint16_t b)
{
#if ARITHMETIC_ASM_AVR8
	__asm__ __volatile__(
		"add %A0, %A1 \n"
		"adc %B0, %B1 \n"
		"brcc 1f \n"
		"ldi %A0, 0xFF \n"
		"ldi %B0, 0xFF \n"
		"1: \n"
		: "+d" (a)
		: "d" (b)
	);
	return a;
#else
	uint16_t x = (uint16_t)(a + b);
	if (x < a)
		return UINT16_MAX;
	return x;
#endif
}

/* Saturated unsigned addition (8 bit). */
static inline uint8_t add_sat_u8(uint8_t a, uint8_t b)
{
#if ARITHMETIC_ASM_AVR8
	__asm__ __volatile__(
		"add %0, %1 \n"
		"brcc 1f \n"
		"ldi %0, 0xFF \n"
		"1: \n"
		: "+d" (a)
		: "d" (b)
	);
	return a;
#else
	uint8_t x = (uint8_t)(a + b);
	if (x < a)
		return UINT8_MAX;
	return x;
#endif
}

/* Saturated unsigned subtraction (32 bit). */
static inline uint32_t sub_sat_u32(uint32_t a, uint32_t b)
{
#if ARITHMETIC_ASM_AVR8
	__asm__ __volatile__(
		"sub %A0, %A1 \n"
		"sbc %B0, %B1 \n"
		"sbc %C0, %C1 \n"
		"sbc %D0, %D1 \n"
		"brcc 1f \n"
		"clr %A0 \n"
		"clr %B0 \n"
		"movw %C0, %A0 \n"
/*		"clr %C0 \n"
		"clr %D0 \n" */
		"1: \n"
		: "+r" (a)
		: "r" (b)
	);
	return a;
#else
	uint32_t x = (uint32_t)(a - b);
	if (x > a)
		return 0;
	return x;
#endif
}

#ifdef uint24_t
/* Saturated unsigned subtraction (24 bit). */
static inline uint24_t sub_sat_u24(uint24_t a, uint24_t b)
{
#if ARITHMETIC_ASM_AVR8
	__asm__ __volatile__(
		"sub %A0, %A1 \n"
		"sbc %B0, %B1 \n"
		"sbc %C0, %C1 \n"
		"brcc 1f \n"
		"clr %A0 \n"
		"clr %B0 \n"
		"clr %C0 \n"
		"1: \n"
		: "+r" (a)
		: "r" (b)
	);
	return a;
#else
	uint24_t x = (uint24_t)(a - b);
	if (x > a)
		return 0;
	return x;
#endif
}
#endif /* uint24_t */

/* Saturated unsigned subtraction (16 bit). */
static inline uint16_t sub_sat_u16(uint16_t a, uint16_t b)
{
#if ARITHMETIC_ASM_AVR8
	if (__builtin_constant_p(b)) {
		__asm__ __volatile__(
			"sbiw %A0, %A1 \n"
			"brcc 1f \n"
			"clr %A0 \n"
			"clr %B0 \n"
			"1: \n"
			: "+r" (a)
			: "M" (b)
		);
	} else {
		__asm__ __volatile__(
			"sub %A0, %A1 \n"
			"sbc %B0, %B1 \n"
			"brcc 1f \n"
			"clr %A0 \n"
			"clr %B0 \n"
			"1: \n"
			: "+r" (a)
			: "r" (b)
		);
	}
	return a;
#else
	uint16_t x = (uint16_t)(a - b);
	if (x > a)
		return 0;
	return x;
#endif
}

/* Saturated unsigned subtraction (8 bit). */
static inline uint8_t sub_sat_u8(uint8_t a, uint8_t b)
{
#if ARITHMETIC_ASM_AVR8
	if (__builtin_constant_p(b)) {
		__asm__ __volatile__(
			"subi %0, %1 \n"
			"brcc 1f \n"
			"clr %0 \n"
			"1: \n"
			: "+r" (a)
			: "M" (b)
		);
	} else {
		__asm__ __volatile__(
			"sub %0, %1 \n"
			"brcc 1f \n"
			"clr %0 \n"
			"1: \n"
			: "+r" (a)
			: "r" (b)
		);
	}
	return a;
#else
	uint8_t x = (uint8_t)(a - b);
	if (x > a)
		return 0;
	return x;
#endif
}


#endif /* ARITHMETIC_H_ */
