/* replay a raw ubertooth-btle dump through cb_btle */
#include <stdio.h>
#include "ubertooth.h"
int main(int argc, char **argv) {
	FILE *fp = fopen(argv[1], "rb");
	if (!fp) { perror(argv[1]); return 1; }
	rx_btle_file(fp);
	fclose(fp);
	return 0;
}
