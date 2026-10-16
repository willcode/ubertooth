# native unit tests for portable firmware code
CFLAGS = -std=gnu99 -O1 -g -Wall -Wextra -Wno-unused-parameter -Wno-unused-variable \
	-Istub -I../bluetooth_rxtx -I../common -I../../host/libubertooth/src

all: test_le test_clock
	./test_le
	./test_clock

test_le: test_le.c ../bluetooth_rxtx/bluetooth_le.c stub/ubertooth.h
	$(CC) $(CFLAGS) -o $@ test_le.c ../bluetooth_rxtx/bluetooth_le.c

test_clock: test_clock.c ../bluetooth_rxtx/ubertooth_clock.h
	$(CC) $(CFLAGS) -o $@ test_clock.c

clean:
	rm -f test_le test_clock

.PHONY: all clean
