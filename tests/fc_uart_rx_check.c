/*
 * Host driver for workloads/flight_controller/src/fc_uart_rx.h (tests/test_fc_uart_rx.py).
 * Compiled as C and as C++ (main.cpp includes the header from C++).
 *
 *   fc_uart_rx_check feed <hex> [<hex> ...]
 *       For each argument: put every byte into the ring as the ISR would, then drain the ring
 *       through the line assembler as uart_cmd_poll() would. Prints "L <line>" per completed
 *       line, then "C ok=.. bad=.. ovr=.. bytes=..".
 *   fc_uart_rx_check stress <n>
 *       A producer thread puts n bytes of a counting sequence while the main thread consumes.
 *       Every entry not flagged FC_RX_LOST must follow its predecessor; kept + overruns == n.
 */
#include "fc_uart_rx.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static struct fc_rx_ring ring;
static struct fc_line_asm lines;

static void drain(void)
{
	int entry;

	while ((entry = fc_rx_ring_get(&ring)) >= 0) {
		const char *line = fc_line_feed(&lines, entry);

		if (line != NULL) {
			printf("L %s\n", line);
		}
	}
}

static int hexval(char c)
{
	if (c >= '0' && c <= '9') {
		return c - '0';
	}
	if (c >= 'a' && c <= 'f') {
		return c - 'a' + 10;
	}
	return -1;
}

static int feed(int argc, char **argv)
{
	for (int a = 0; a < argc; ++a) {
		const size_t n = strlen(argv[a]);

		for (size_t i = 0; i + 1 < n; i += 2) {
			const int hi = hexval(argv[a][i]);
			const int lo = hexval(argv[a][i + 1]);

			if (hi < 0 || lo < 0) {
				fprintf(stderr, "bad hex\n");
				return 2;
			}
			fc_rx_ring_put(&ring, (uint8_t)(hi * 16 + lo));
		}
		drain();
	}
	printf("C ok=%u bad=%u ovr=%u bytes=%u\n", (unsigned)lines.lines_ok,
	       (unsigned)lines.lines_bad, (unsigned)fc_rx_ring_overruns(&ring),
	       (unsigned)fc_rx_ring_bytes(&ring));
	return 0;
}

static unsigned long stress_n;
static volatile int producer_done;

static void *producer(void *arg)
{
	(void)arg;
	for (unsigned long i = 0; i < stress_n; ++i) {
		fc_rx_ring_put(&ring, (uint8_t)(i & 0xffu));
	}
	__atomic_store_n(&producer_done, 1, __ATOMIC_RELEASE);
	return NULL;
}

static int stress(unsigned long n)
{
	pthread_t thread;
	unsigned long kept = 0;
	int prev = -1;
	int entry;

	stress_n = n;
	if (pthread_create(&thread, NULL, producer, NULL) != 0) {
		fprintf(stderr, "pthread_create failed\n");
		return 2;
	}
	for (;;) {
		const int done = __atomic_load_n(&producer_done, __ATOMIC_ACQUIRE);

		while ((entry = fc_rx_ring_get(&ring)) >= 0) {
			const int byte = entry & 0xff;

			if (prev >= 0 && !(entry & (int)FC_RX_LOST) && byte != ((prev + 1) & 0xff)) {
				printf("ORDER kept=%lu prev=%d got=%d\n", kept, prev, byte);
				return 1;
			}
			prev = byte;
			kept++;
		}
		if (done) {
			break;
		}
	}
	pthread_join(thread, NULL);
	printf("S kept=%lu ovr=%u bytes=%u\n", kept, (unsigned)fc_rx_ring_overruns(&ring),
	       (unsigned)fc_rx_ring_bytes(&ring));
	return 0;
}

int main(int argc, char **argv)
{
	if (argc >= 2 && strcmp(argv[1], "feed") == 0) {
		return feed(argc - 2, argv + 2);
	}
	if (argc == 3 && strcmp(argv[1], "stress") == 0) {
		return stress(strtoul(argv[2], NULL, 10));
	}
	fprintf(stderr, "usage: %s feed <hex>... | stress <n>\n", argv[0]);
	return 2;
}
