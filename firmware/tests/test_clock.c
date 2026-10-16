/* unit tests for the firmware's CLKN tick arithmetic, compiled natively:
 *   make -C firmware/tests
 */
#include <stdio.h>
#include "ubertooth_clock.h"

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++failures; } } while (0)

int main(void) {
	/* the nominal tick, and a trim out of range, leave the match value alone */
	CHECK(clkn_tick_match(0, 0) == 3124);
	CHECK(clkn_tick_match(0, 3000) == 3124);
	CHECK(clkn_tick_match(6250, 0) == 3124);
	CHECK(clkn_tick_match(65535, 0) == 3124);

	/* a delay stretches the tick by the offset */
	CHECK(clkn_tick_match(1, 0) == 3125);
	CHECK(clkn_tick_match(3124, 0) == 6248);

	/* an advance shortens it: 6249 by one count, 3125 by a whole tick, which the
	 * clamp turns into the shortest tick allowed rather than 0xFFFFFFFF */
	CHECK(clkn_tick_match(6249, 0) == 3123);
	CHECK(clkn_tick_match(3143, 0) == 17);
	CHECK(clkn_tick_match(3142, 0) == 16);
	CHECK(clkn_tick_match(3141, 0) == 16);
	CHECK(clkn_tick_match(3126, 0) == 16);
	CHECK(clkn_tick_match(3125, 0) == 16);
	CHECK(clkn_tick_match(3125, 100) == 116);
	CHECK(clkn_tick_match(5000, 3000) == 3016);

	/* every trim, at every count into the tick: in range and never behind the count */
	for (uint32_t off = 0; off < 6250; ++off) {
		for (uint32_t tc = 0; tc <= 3124; tc += 7) {
			const uint32_t mr0 = clkn_tick_match(off, tc);
			CHECK(mr0 >= 16 && mr0 <= 6248);
			if (off > 3124 && off < 6250)
				CHECK(mr0 >= tc + 16);
			else
				CHECK(mr0 >= 3124);
		}
	}

	if (failures == 0)
		printf("all clock tests passed\n");
	return failures != 0;
}
