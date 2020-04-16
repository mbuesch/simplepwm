#include <stdint.h>
#include <stdio.h>
#define __int24 int32_t
#define __uint24 uint32_t
#define cli()
#define sei()
#define COMPAT_H_
uint8_t SREG;

#include "../color.c"

int main(int argc, char **argv)
{
	uint16_t r, g, b;
	unsigned int h, s, l;

	sscanf(argv[1], "%u", &h);
	sscanf(argv[2], "%u", &s);
	sscanf(argv[3], "%u", &l);
	hsl2rgb(&r, &g, &b, h, s, l);
	printf("%u %u %u\n", r, g, b);
}
