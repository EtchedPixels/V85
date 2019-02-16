/*
 *	Platform features
 *
 *	8085 at 6MHz
 *	Motorola 6850 at 0x01/0x02
 *	IDE at 0x10-0x17 no high or control access
 *	Page select at 0x40
 *	Interrupt/timer at 0xFE
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include "intel_8085_emulator.h"
#include "ide.h"

static uint8_t baseram[16384];
static uint8_t bankram[8][49152];

static uint8_t fast = 0;
static uint8_t banknum;

static uint16_t tstate_steps = 300;	/* Set me properly */

#define TRACE_MEM	1
#define TRACE_IO	2
#define TRACE_ROM	4
#define TRACE_UNK	8
#define TRACE_SIO	16
#define TRACE_512	32
#define TRACE_RTC	64
#define TRACE_ACIA	128
#define TRACE_CTC	256
#define TRACE_CPLD	512

static int trace = 0;


uint8_t i8085_read(uint16_t addr)
{
	uint8_t dummy = 0xFF;
	uint8_t *p = bankram[banknum] + addr;
	if (addr >= 0xC000)
		p = baseram + (addr & 0x3FFF);
	else if (banknum == 8) 	/* Invalid setting */
		p = &dummy;
	if (trace & TRACE_MEM)
		fprintf(stderr, "R[%d] %04X = %02X\n", banknum, addr, *p);
	return *p;
}

void i8085_write(uint16_t addr, uint8_t val)
{
	uint8_t *p = bankram[banknum] + addr;
	if (addr >= 0xC000)
		p = baseram + (addr & 0x3FFF);
	else if (banknum == 8) {
		if (trace & TRACE_MEM)
			fprintf(stderr, "W[%d] %04X: bad bank.\n",
					banknum, addr);
		return;
	}
	*p = val;
	if (trace & TRACE_MEM)
		fprintf(stderr, "W%d %04X = %02X\n", banknum, addr, val);
}

static int check_chario(void)
{
	fd_set i, o;
	struct timeval tv;
	unsigned int r = 0;

	FD_ZERO(&i);
	FD_SET(0, &i);
	FD_ZERO(&o);
	FD_SET(1, &o);
	tv.tv_sec = 0;
	tv.tv_usec = 0;

	if (select(2, &i, NULL, NULL, &tv) == -1) {
		if (errno == EINTR)
			return 0;
		perror("select");
		exit(1);
	}
	if (FD_ISSET(0, &i))
		r |= 1;
	if (FD_ISSET(1, &o))
		r |= 2;
	return r;
}

static unsigned int next_char(void)
{
	char c;
	if (read(0, &c, 1) != 1) {
		printf("(tty read without ready byte)\n");
		return 0xFF;
	}
	if (c == 0x0A)
		c = '\r';
	return c;
}

static uint8_t acia_status = 2;
static uint8_t acia_config;
static uint8_t acia_char;
static uint8_t acia_inint = 0;

static void acia_irq_compute(void)
{
	if (acia_config & acia_status & 0x80) {
		if (acia_inint)
			return;
		if (trace & TRACE_ACIA)
			fprintf(stderr, "ACIA interrupt.\n");
		acia_inint = 1;
		i8085_set_int(INT_RST75);
	}
}

static void acia_receive(void)
{
	uint8_t old_status = acia_status;
	acia_status = old_status & 0x02;
	if (old_status & 1)
		acia_status |= 0x20;
	acia_char = next_char();
	if (trace & TRACE_ACIA)
		fprintf(stderr, "ACIA rx.\n");
	acia_status |= 0x81;	/* IRQ, and rx data full */
}

static void acia_transmit(void)
{
	if (!(acia_status & 2)) {
		if (trace & TRACE_ACIA)
			fprintf(stderr, "ACIA tx is clear.\n");
		acia_status |= 0x82;	/* IRQ, and tx data empty */
	}
}

static void acia_timer(void)
{
	int s = check_chario();
	if (s & 1)
		acia_receive();
	if (s & 2)
		acia_transmit();
	if (s)
		acia_irq_compute();
}

/* Very crude for initial testing ! */
static uint8_t acia_read(uint8_t addr)
{
	if (trace & TRACE_ACIA)
		fprintf(stderr, "acia_read %d ", addr);
	switch (addr) {
	case 0:
		/* bits 7: irq pending, 6 parity error, 5 rx over
		 * 4 framing error, 3 cts, 2 dcd, 1 tx empty, 0 rx full.
		 * Bits are set on char arrival and cleared on next not by
		 * user
		 */
		acia_status &= ~0x80;
		acia_inint = 0;
		if (trace & TRACE_ACIA)
			fprintf(stderr, "acia_status %d\n", acia_status);
		return acia_status;
	case 1:
		acia_status &= ~0x81;	/* No IRQ, rx empty */
		acia_inint = 0;
		if (trace & TRACE_ACIA)
			fprintf(stderr, "acia_char %d\n", acia_char);
		return acia_char;
	default:
		fprintf(stderr, "acia: bad addr.\n");
		exit(1);
	}
}

static void acia_write(uint16_t addr, uint8_t val)
{
	if (trace & TRACE_ACIA)
		fprintf(stderr, "acia_write %d %d\n", addr, val);
	switch (addr) {
	case 0:
		/* bit 7 enables interrupts, bits 5-6 are tx control
		   bits 2-4 select the word size and 0-1 counter divider
		   except 11 in them means reset */
		acia_config = val;
		if ((acia_config & 3) == 3) {
			acia_status = 2;
			acia_inint = 0;
		}
		acia_irq_compute();
		return;
	case 1:
		write(1, &val, 1);
		/* Clear any existing int state and tx empty */
		acia_status &= ~0x82;
		break;
	}
}
struct ide_controller *ide0;

static uint8_t my_ide_read(uint16_t addr)
{
	return ide_read8(ide0, addr);
}

static void my_ide_write(uint16_t addr, uint8_t val)
{
	ide_write8(ide0, addr, val);
}

static uint8_t timer_val;
static uint8_t timer_count;

static uint8_t timer_read(void)
{
	return timer_val;
}

static void timer_write(uint8_t val)
{
	timer_val = val;
	if (timer_val & 0x50)
		i8085_clear_int(INT_RST65);
}

static void timer_tick(void)
{
	timer_count++;
	if (timer_count < 20)
		return;
	timer_count = 0;
	if (timer_val & 0x40)
		i8085_set_int(INT_RST65);
}

static void bank_write(uint8_t bank)
{
	switch(bank) {
	case 0:
		banknum = 8;
		break;
	case 1:
		banknum = 0;
		break;
	case 2:
		banknum = 1;
		break;
	case 4:
		banknum = 2;
		break;
	case 8:
		banknum = 3;
		break;
	case 16:
		banknum = 4;
		break;
	case 32:
		banknum = 5;
		break;
	case 64:
		banknum = 6;
		break;
	case 128:
		banknum = 7;
		break;
	default:
		fprintf(stderr, "Invalid bank setting %02X\n", bank);
		banknum = 8;
		break;
	}
}

uint8_t i8085_inport(uint8_t addr)
{
	if (trace & TRACE_IO)
		fprintf(stderr, "read %02x\n", addr);
	if (addr >= 0x00 && addr <= 0x01)
		return acia_read(addr & 1);
	if (addr >= 0x10 && addr <= 0x17)
		return my_ide_read(addr & 7);
	if (addr == 0xFE)
		timer_read();
	if (trace & TRACE_UNK)
		fprintf(stderr, "Unknown read from port %02X\n", addr);
	return 0xFF;
}

void i8085_outport(uint8_t addr, uint8_t val)
{
	if (trace & TRACE_IO)
		fprintf(stderr, "write %02x <- %02x\n", addr, val);
	if (addr >= 0x00 && addr <= 0x01)
		acia_write(addr & 1, val);
	else if (addr >= 0x10 && addr <= 0x17)
		my_ide_write(addr & 7, val);
	else if (addr == 0x40)
		bank_write(val);
	else if (addr == 0xFD) {
		printf("trace set to %d\n", val);
		trace = val;
	} else if (addr == 0xFE) {
		timer_write(val);
	} else if (trace & TRACE_UNK)
		fprintf(stderr, "Unknown write to port %04X of %02X\n", addr, val);
}

static struct termios saved_term, term;

static void cleanup(int sig)
{
	tcsetattr(0, TCSADRAIN, &saved_term);
	exit(1);
}

static void exit_cleanup(void)
{
	tcsetattr(0, TCSADRAIN, &saved_term);
}

static void usage(void)
{
	fprintf(stderr, "v85: [-f] [-d debug]\n");
	exit(EXIT_FAILURE);
}

int main(int argc, char *argv[])
{
	static struct timespec tc;
	int opt;
	int fd;

	while ((opt = getopt(argc, argv, "d:f")) != -1) {
		switch (opt) {
		case 'd':
			trace = atoi(optarg);
			break;
		case 'f':
			fast = 1;
			break;
		default:
			usage();
		}
	}
	if (optind < argc)
		usage();

	fd = open("v85.rom", O_RDONLY);
	if (fd == -1) {
		perror("v85.rom");
		exit(EXIT_FAILURE);
	}
	if (read(fd, bankram[0], 512) < 8) {
		fprintf(stderr, "v85: short rom 'v85.rom'.\n");
		exit(EXIT_FAILURE);
	}
	close(fd);

	ide0 = ide_allocate("cf");
	if (ide0) {
		int ide_fd = open("v85.ide", O_RDWR);
		if (ide_fd == -1) {
			perror("v85.ide");
			exit(1);
		}
		if (ide_attach(ide0, 0, ide_fd) == 0)
			ide_reset_begin(ide0);
	} else {
		fprintf(stderr, "v85: ide set up failed.\n");
		exit(1);
	}

	/* 5ms - it's a balance between nice behaviour and simulation
	   smoothness */
	tc.tv_sec = 0;
	tc.tv_nsec = 5000000L;

	if (tcgetattr(0, &term) == 0) {
		saved_term = term;
		atexit(exit_cleanup);
		signal(SIGINT, cleanup);
		signal(SIGQUIT, cleanup);
		signal(SIGPIPE, cleanup);
		term.c_lflag &= ~(ICANON | ECHO);
		term.c_cc[VMIN] = 0;
		term.c_cc[VTIME] = 1;
		term.c_cc[VINTR] = 0;
		term.c_cc[VSUSP] = 0;
		term.c_cc[VSTOP] = 0;
		tcsetattr(0, TCSADRAIN, &term);
	}

	i8085_reset();

	/* This is the wrong way to do it but it's easier for the moment. We
	   should track how much real time has occurred and try to keep cycle
	   matched with that. The scheme here works fine except when the host
	   is loaded though */

	/* We run 300 cycles per I/O check, do that 100 times then poll the
	   slow stuff and nap for 5ms. */
	/* FIXME: proper timings */
	while (1) {
		int i;
		/* 36400 T states */
		for (i = 0; i < 100; i++) {
			i8085_exec(tstate_steps);
			acia_timer();
		}
		/* Do 5ms of I/O and delays */
		if (!fast)
			nanosleep(&tc, NULL);
		timer_tick();
	}
	exit(0);
}
