/*
 * The HackRF LE search on a word stream from gen_hle_words.py: every
 * expected packet found once, with its direction and aux flag, and nothing
 * else.
 *
 * usage: test_hackrf_le words.u16 expected.txt [center [bank to drop]]
 *
 * This file is part of Project Ubertooth (GPL-2.0-or-later).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ubertooth_hackrf_le.h"

#define MAX_EXPECTED 64

typedef struct {
	int freq;
	unsigned long long word;
	int slave, aux;
	char pdu[2 * HLE_MAX_BYTES + 1];
	int found;
} expected_t;

static expected_t exp_[MAX_EXPECTED];
static int n_exp, errors;

static void on_packet(void* user, const hle_packet* p)
{
	char hex[2 * HLE_MAX_BYTES + 1];
	unsigned i;
	int j;
	(void)user;

	/* the PDU without the access address and the CRC */
	for (i = 4; i + 3 < p->len; i++)
		sprintf(hex + 2 * (i - 4), "%02x", p->data[i]);
	hex[2 * (p->len - 7)] = '\0';
	printf("found %d MHz word %llu aa %08x errors %d %s%s %s\n", p->freq,
	       (unsigned long long)p->word, p->aa, p->aa_errors,
	       p->is_data ? (p->slave ? "S->M" : "M->S") : "adv", p->aux ? " aux" : "", hex);
	for (j = 0; j < n_exp; j++) {
		expected_t* e = &exp_[j];
		if (e->freq != p->freq || e->word != p->word || strcmp(e->pdu, hex) != 0)
			continue;
		if (e->found++)
			printf("  ERROR: found twice\n"), errors++;
		if (e->slave != p->slave || e->aux != p->aux)
			printf("  ERROR: expected slave %d aux %d\n", e->slave, e->aux), errors++;
		return;
	}
	printf("  ERROR: not expected\n");
	errors++;
}

int main(int argc, char** argv)
{
	FILE* f;
	uint16_t bank[400];
	unsigned long long start = 0;
	int center = argc > 3 ? atoi(argv[3]) : 2409, j;
	long drop = argc > 4 ? atol(argv[4]) : -1, bankno = 0;
	size_t n;
	hle_stats st;

	if (argc < 3) {
		fprintf(stderr, "usage: %s words.u16 expected.txt [center [bank to drop]]\n", argv[0]);
		return 2;
	}
	f = fopen(argv[2], "r");
	if (f == NULL) {
		perror(argv[2]);
		return 2;
	}
	while (n_exp < MAX_EXPECTED &&
	       fscanf(f, "%d %llu %d %d %s", &exp_[n_exp].freq, &exp_[n_exp].word,
	              &exp_[n_exp].slave, &exp_[n_exp].aux, exp_[n_exp].pdu) == 5)
		n_exp++;
	fclose(f);

	f = fopen(argv[1], "rb");
	if (f == NULL) {
		perror(argv[1]);
		return 2;
	}
	hle_init(on_packet, NULL, 2, 1);
	while ((n = fread(bank, sizeof(bank[0]), 400, f)) > 0) {
		if (bankno++ != drop)
			hle_bank(bank, (int)n, start, center);
		start += n;
	}
	fclose(f);

	for (j = 0; j < n_exp; j++)
		if (!exp_[j].found)
			printf("ERROR: missed %d MHz word %llu %s\n", exp_[j].freq, exp_[j].word,
			       exp_[j].pdu), errors++;
	hle_get_stats(&st);
	printf("%lu matches, %lu good, %lu bad CRC, %lu connections; %d expected, %d errors\n",
	       st.matches, st.crc_ok, st.crc_bad, st.connections, n_exp, errors);
	return errors ? 1 : 0;
}
