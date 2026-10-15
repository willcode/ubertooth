/* test the BLE connection recovery solver against simulated observations */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ubertooth_le_recover.h"

static int hexbyte(const char *s) { unsigned v; sscanf(s, "%2x", &v); return v; }

static int use_truth_map;
static int run(const char *path) {
	FILE *f = fopen(path, "r");
	char line[1024];
	unsigned aa, crc, csa, interval, hop, counter, unmapped;
	le_recover_t r;
	int ok = 1;

	if (!f) { perror(path); return 0; }
	fgets(line, sizeof line, f);
	fgets(line, sizeof line, f);
	unsigned long long chm_bits = 0x1fffffffffull;
	sscanf(line, "%x %x %u %u %u %u %u %llx", &aa, &crc, &csa, &interval, &hop, &counter, &unmapped, &chm_bits);
	uint8_t chm[5] = { chm_bits, chm_bits >> 8, chm_bits >> 16, chm_bits >> 24, chm_bits >> 32 };
	le_recover_init(&r, aa);
	while (fgets(line, sizeof line, f)) {
		unsigned ts, ch; char hex[600]; uint8_t pdu[300]; unsigned n, i;
		if (sscanf(line, "%u %u %599s", &ts, &ch, hex) != 3) continue;
		n = strlen(hex) / 2;
		for (i = 0; i < n; ++i) pdu[i] = hexbyte(hex + 2 * i);
		le_recover_packet(&r, ts, ch, pdu, n);
	}
	fclose(f);

	uint32_t got_crc = 0;
	unsigned votes = le_recover_get_crc(&r, &got_crc);
	uint16_t iv = le_recover_interval(&r, 8);
	le_follow_params_t p;
	le_recover_quality_t q;
	int solved = use_truth_map ? le_recover_solve_map(&r, iv, chm, &p, &q) : le_recover_solve(&r, iv, &p, &q);
	const char *how = use_truth_map ? "known map" : "assumed map";
	// CSA#1 with a partial map needs a surveyed map, as ubertooth-btle does
	if (!solved && !use_truth_map) {
		solved = le_recover_solve_map(&r, iv, chm, &p, &q);
		how = "surveyed map";
	}

	printf("%-16s events=%3u chans=%u crc %06x(%u votes)%s interval %u(%s) ", path, r.num_events,
		le_recover_channels_seen(&r), got_crc, votes, got_crc == crc ? "" : " WRONG", iv,
		iv == interval || iv == interval * 37 ? "ok" : "WRONG");
	ok &= got_crc == crc;
	if (!solved) {
		printf("UNSOLVED\n");
		return 0;
	}
	printf("csa%d %s matched %u/%u (runner-up %u) ", p.csa, how, q.matched, q.considered, q.runner_up);
	ok &= p.csa == (int)csa && p.interval == interval;
	if (csa == 2)
		ok &= p.counter == counter;
	else
		ok &= p.hop == hop && p.unmapped == unmapped && p.counter == counter % 37;
	printf("%s [got hop %u unmapped %u counter %u]\n", ok ? "PASS" : "FAIL", p.hop, p.unmapped, p.counter);
	return ok;
}

int main(int argc, char **argv) {
	int i, ok = 1;
	if (argc > 1 && strcmp(argv[1], "-m") == 0) { use_truth_map = 1; ++argv; --argc; }
	for (i = 1; i < argc; ++i)
		ok &= run(argv[i]);
	return !ok;
}
