/* unit tests for firmware LE helpers, compiled natively:
 *   make -C firmware/tests
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bluetooth_le.h"

uint8_t le_csa1_channel(uint16_t counter, uint8_t hop_increment,
		le_channel_remapping_t *remapping);
uint16_t le_csa2_prn_e(uint16_t counter, uint16_t channel_id);
uint8_t le_csa2_channel(uint16_t counter, uint16_t channel_id,
		le_channel_remapping_t *remapping);

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++failures; } } while (0)

static void map_from_list(uint8_t *chm, const int *chans, int n) {
	memset(chm, 0, 5);
	for (int i = 0; i < n; ++i)
		chm[chans[i] / 8] |= 1 << (chans[i] % 8);
}

int main(void) {
	le_channel_remapping_t r;
	uint8_t chm[5];

	/* CSA#2 sample data: Core Spec Vol 6 Part C 3.1 */
	uint32_t aa = 0x8E89BED6;
	uint16_t chan_id = (aa >> 16) ^ (aa & 0xffff);
	CHECK(chan_id == 0x305F);
	uint8_t full[5] = { 0xff, 0xff, 0xff, 0xff, 0x1f };
	le_parse_channel_map(full, &r);
	CHECK(r.total_channels == 37);
	CHECK(le_csa2_channel(0, chan_id, &r) == 25);
	CHECK(le_csa2_channel(1, chan_id, &r) == 20);
	CHECK(le_csa2_channel(2, chan_id, &r) == 6);
	CHECK(le_csa2_channel(3, chan_id, &r) == 21);
	int part[] = { 9, 10, 21, 22, 23, 33, 34, 35, 36 };
	map_from_list(chm, part, 9);
	le_parse_channel_map(chm, &r);
	CHECK(r.total_channels == 9);
	CHECK(le_csa2_channel(6, chan_id, &r) == 23);
	CHECK(le_csa2_channel(7, chan_id, &r) == 9);
	CHECK(le_csa2_channel(8, chan_id, &r) == 34);

	/* CSA#1 as a function of the counter matches the iterative form */
	int part2[] = { 1, 4, 7, 12, 20, 30, 36 };
	map_from_list(chm, part2, 7);
	le_parse_channel_map(chm, &r);
	unsigned last = 0;
	for (unsigned n = 0; n < 1000; ++n) {
		last = (last + 11) % 37;
		uint8_t want = r.channel_in_use[last] ? last
			: r.remapping_index[last % r.total_channels];
		CHECK(le_csa1_channel(n, 11, &r) == want);
	}

	/* fast dewhitening matches the original bit-at-a-time algorithm */
	for (unsigned ch = 2402; ch <= 2480; ch += 2) {
		uint8_t in[260], fast[260], ref[260];
		for (unsigned i = 0; i < sizeof(in); ++i)
			in[i] = (uint8_t)(i * 37 + ch);
		memcpy(fast, in, sizeof(in));
		le_dewhiten(fast, sizeof(fast), ch);
		unsigned idx = whitening_index[btle_channel_index(ch)];
		for (unsigned i = 0; i < sizeof(in); ++i) {
			uint8_t out = 0;
			for (unsigned j = 0; j < 8; ++j) {
				unsigned bit = ((in[i] >> (7 - j)) & 1) ^ whitening[idx];
				idx = (idx + 1) % sizeof(whitening);
				out |= bit << j;
			}
			ref[i] = out;
		}
		CHECK(memcmp(fast, ref, sizeof(ref)) == 0);
		for (unsigned off = 0; off < 4; ++off)
			CHECK(le_dewhiten_byte(ch, off, in[off]) == ref[off]);
	}

	/* CRC of an ADV_NONCONN_IND captured by an Ubertooth */
	uint8_t adv[] = { 0x42, 0x0e, 0x79, 0x5a, 0x62, 0x12, 0xb3, 0xd3, 0x07, 0xff,
		0x4c, 0x00, 0x12, 0x02, 0x00, 0x03, 0xf1, 0xb5, 0xee };
	CHECK(le_reverse_crc_init(0x555555) == 0xAAAAAA);
	CHECK(le_crc_ok(0xAAAAAA, adv, sizeof(adv)));
	adv[5] ^= 1;
	CHECK(!le_crc_ok(0xAAAAAA, adv, sizeof(adv)));

	if (failures == 0)
		printf("all LE tests passed\n");
	return failures != 0;
}
