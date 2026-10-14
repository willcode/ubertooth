/* host-side stub of firmware/common/ubertooth.h for unit tests */
#ifndef STUB_UBERTOOTH_H
#define STUB_UBERTOOTH_H
#include <stdint.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int8_t s8;
static inline u32 rbit(u32 v) {
	u32 r = 0;
	for (int i = 0; i < 32; ++i)
		r |= ((v >> i) & 1u) << (31 - i);
	return r;
}
#endif
