/*
 *	Platform features
 *
 *	8085 at 6MHz with some interrupt lines directly wired
 *	Motorola 6850 ACIA at 0x00/0x01
 *	IDE at 0x10-0x17 no high or control access no interrupt
 *	NEC765 FDC at 0x18-0x1F no interrupt (currently)
 *	I/O based RAMdrive at 0xC6/C7 (CompuPro M)
 *	Dual Systems CLK-24 at 0xF0/F1
 *
 *
 *	Page select at 0x40
 *	Interrupt/timer at 0xFE
 *
 *	Possible additions to consider
 *	- Compupro Disk 3 or a generic ST506 disk interface
 *
 *	In progress
 *	- ALT256
 *	- DMA on the FDC
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
#include "765.h"

static volatile uint8_t done;

static uint8_t baseram[16384];
static uint8_t bankram[8][49152];
static uint8_t rom[512];

static uint8_t fast = 0;
static uint8_t banknum = 8;	/* bank reg starts 0 */
static uint8_t bankmap = 0x0f;

static FDC_PTR fdc;
static FDRV_PTR drive_a, drive_b, drive_c;


/* We do 6MHz so 6,000,000 tstates a second. That works out at 30,000 per
   5ms working period, each of which we split 200 ways */

static uint16_t tstate_steps = 150;

#define TRACE_MEM	1
#define TRACE_IO	2
#define TRACE_UNK	8
#define TRACE_ACIA	16
#define TRACE_BANK	32
#define TRACE_FDC	64
#define TRACE_DMA	128
#define TRACE_MDRIVE	256
#define TRACE_RTC	512
#define TRACE_CPU	1024

static int trace = 0;


uint8_t i8085_debug_read(uint16_t addr)
{
	uint8_t *p = bankram[banknum] + addr;
	if (addr >= 0xC000)
		p = baseram + (addr & 0x3FFF);
	else if (banknum == 8) 	/* ROM */
		p = rom + (addr & 0x1FF);
	return *p;
}

uint8_t i8085_read(uint16_t addr)
{
	uint8_t *p = bankram[banknum] + addr;
	if (addr >= 0xC000)
		p = baseram + (addr & 0x3FFF);
	else if (banknum == 8) 	/* ROM */
		p = rom + (addr & 0x1FF);
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
			fprintf(stderr, "W[%d] %04X: ROM write.\n",
					banknum, addr);
		return;
	}
	*p = val;
	if (trace & TRACE_MEM)
		fprintf(stderr, "W%d %04X = %02X\n", banknum, addr, val);
}

/* We don't currently use the RIM/SIM driven GPIO pins */

int i8085_get_input(void)
{
	return 0;
}

void i8085_set_output(int val)
{
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

/*
 *	6850 at 0/1. A fairly common setup. The only oddity is we use
 *	the 8085 interrupt lines.
 */

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
		acia_irq_compute();
		break;
	}
}

/*
 *	Modern 8bit IDE adapter (it wouldn't be hard to tweak this to be
 *	a more period appropriate ST506 interface..)
 */
struct ide_controller *ide0;

static uint8_t my_ide_read(uint16_t addr)
{
	return ide_read8(ide0, addr);
}

static void my_ide_write(uint16_t addr, uint8_t val)
{
	ide_write8(ide0, addr, val);
}

/*
 *	Intel 8237
 */

struct i8237_channel {
	uint16_t bar;
	uint16_t bwcr;
	uint16_t car;
	uint16_t cwcr;
	uint8_t mode;
};

struct i8237 {
	struct i8237_channel chan[4];
	uint16_t tar;
	uint16_t tacr;
	uint8_t status;
	uint8_t command;
	uint8_t temp;
	uint8_t mask;
	uint8_t request;
	uint8_t flipflop;
};

static struct i8237 i8237;

static int i8237_fdc_dma;


static void i8237_inc(struct i8237_channel *c)
{
	if(c->mode & 0x20)
		c->car--;
	else
		c->car++;
}

static void i8237_count(struct i8237 *dmac, int chan, struct i8237_channel *c)
{
	c->cwcr--;
	if (c->cwcr == 0xFFFF) {
		/* Set terminal count */
		dmac->mask |= (0x10 << chan);
		/* Need to hook FDC here */
		if (chan == 3)
			fdc_set_terminal_count(fdc, 1);
	}
	if (c->mode & 0x10) {	/* Autoinit */
		c->car = c->bar;
		c->cwcr = c->bwcr;
	}
}

static int i8237_cycle(struct i8237 *dmac, int chan)
{
	struct i8237_channel *c = dmac->chan + chan;

	/* Device disable */
	if (dmac->command & 0x04)
		return 0;
	/* Channel disable */
	if (dmac->mask & (0x01 << chan))
		return 0;
	/* DMA is complete */
	if (dmac->status & (0x10 << chan))
		return 0;

	/* We are in business but what are we doing ? */

	/* Memory to memory special case */
	if (dmac->request & 1) {
		/* Memory to memory, source */
		if (chan == 0 && (dmac->command & 1)) {
			dmac->temp = i8085_read(c->car);
			if (!(dmac->command & 2))
				i8237_inc(c);
			/* We don't set tc etc on source */
			c->cwcr--;
			return 4;
		}
		/* Memory to memory, dest: process here */
		if (chan == 1 && (dmac->command & 1)) {
			i8085_write(c->car, dmac->temp);
			i8237_inc(c);
			i8237_count(dmac, chan, c);
			return 4;
		}
	}
	/* We are not doing memory to memory */

	/* For now we have no devices connected to anything but channel 3 */
	if (chan != 3)
		return 0;	/* No activity ever happens */


	/* No FDC data ready to DMA */
	if (i8237_fdc_dma == 0)
		return 0;

	/* We can do a real DMA data transfer */
	i8237_fdc_dma = 0;

	switch (c->mode & 0x0C) {
	case 0x00:
		/* Verify - noop */
		fdc_read_data(fdc);
		break;
	case 0x04:
		fdc_write_data(fdc, i8085_read(c->car));
		break;
	case 0x08:
		i8085_write(c->car, fdc_read_data(fdc));
		break;
	case 0x0C:
		/* Chained - invalid */
		return 0;
	}
	i8237_inc(c);
	i8237_count(dmac, chan, c);
	return 4;
}

/* Run the DMA engine whilst there is work to do */
static int i8237_execute(int ncycl)
{
	int cycles;

	if ((fdc_read_ctrl(fdc) & 0x90) == 0x90)
		i8237_fdc_dma = 1;
	else
		i8237_fdc_dma = 0;

	/* Floppy RQ check once here then once in the cycle set */
	do {
		cycles = 0;
		cycles += i8237_cycle(&i8237, 0);
		cycles += i8237_cycle(&i8237, 1);
		cycles += i8237_cycle(&i8237, 2);
		cycles += i8237_cycle(&i8237, 3);
	} while (cycles && cycles < ncycl);
	return ncycl - cycles;
}

static uint8_t i8237_read(uint8_t addr)
{
	struct i8237_channel *c = i8237.chan + ((addr >> 1) & 0x03);
	uint8_t r;

	switch (addr & 0x0F) {
	case 0x00:
	case 0x02:
	case 0x04:
	case 0x06:
		r = c->car >> (8 * i8237.flipflop);
		i8237.flipflop ^= 1;
		return r;
	case 0x01:
	case 0x03:
	case 0x05:
	case 0x07:
		if (i8237.flipflop) {
			i8237.flipflop = 0;
			return c->cwcr >> 8;
		} else {
			i8237.flipflop = 1;
			return c->cwcr;
		}
	case 0x08:
		/* Hook fdc for chan 3 */
		r = i8237.status;
		i8237.status &= 0x0F;
		fdc_set_terminal_count(fdc, 0);
		return r;
	case 0x0D:
		return i8237.temp;
	/* TODO: check if mode register reading and mode reset is an extension
	   ditto set pointer */
	}
	return 0xFF;
}

static void i8237_write(uint8_t addr, uint8_t val)
{
	struct i8237_channel *c = i8237.chan + ((addr >> 1) & 0x03);

	switch(addr & 0x0F) {
		case 0x00:
		case 0x02:
		case 0x04:
		case 0x06:
			/* Set c->bar */
			if (i8237.flipflop) {
				c->bar &= 0xFF;
				c->bar |= (((uint16_t) val) << 8);
			} else {
				c->bar &= 0xFF00;
				c->bar |= val;
			}
			c->car = c->bar;
			i8237.flipflop ^= 1;
			break;
		case 0x01:
		case 0x03:
		case 0x05:
		case 0x07:
			/* set c->bwcr */
			if (i8237.flipflop) {
				c->bwcr &= 0xFF;
				c->bwcr |= (((uint16_t) val) << 8);
			} else {
				c->bwcr &= 0xFF00;
				c->bwcr |= val;
			}
			c->cwcr = c->bwcr;
			i8237.flipflop ^= 1;
			break;
		case 0x08:
			i8237.command = val;
			break;
		case 0x09:
			i8237.request = val|0xF0;
			break;
		case 0x0A:
			/* Write single mask register bit */
			if (val & 4)
				i8237.mask |= 1 << (val & 3);
			else
				i8237.mask &= ~(1 << (val & 3));
			break;
		case 0x0B:
			i8237.chan[val & 3].mode = val;
			break;
		case 0x0C:
			/* Clear byte pointer flipflop */
			i8237.flipflop = 0;
			break;
		case 0x0D:
			/* Master clear */
			i8237.flipflop = 0;
			i8237.command = 0;
			i8237.status = 0;
			i8237.request = 0xF0;
			i8237.temp = 0;
			i8237.mask = 0x0F;
			break;
		case 0x0E:
			/* Clear mask register */
			i8237.mask = 0x00;
			break;
		case 0x0F:
			/* Write all mask register bits */
			i8237.mask &= 0xF0;
			i8237.mask |= val & 0x0F;	/* ??? */
			break;
	}
}

/*
 *	Classic NEC765A style floppy disk interface with 5.25" Drives
 */
static uint8_t fdc_ctrl;

static uint8_t fdc_read(uint8_t addr)
{
	switch(addr & 0x03) {
	case 0:
		return fdc_read_data(fdc);
	case 1:
		return fdc_read_ctrl(fdc);
	case 3:
		return fdc_ctrl;
	}
	return 0xFF;
}

static void fdc_write(uint8_t addr, uint8_t val)
{
	addr &= 3;
	switch(addr) {
	case 0:
		fdc_write_data(fdc, val);
		break;
	case 1:
		break;
	case 2:
		fdc_set_terminal_count(fdc, val & 0x80);
		break;
	case 3:
		fdc_ctrl = val;
		fdc_set_motor(fdc, (val & 0x01) ? 0x0F: 0x00);
		break;
	}
}

static void fdc_log(int debuglevel, char *fmt, va_list ap)
{
	if ((trace & TRACE_FDC) || debuglevel == 0)
		vfprintf(stderr, "fdc: ", ap);
}

/*
 *	CompuPro M Drive alike
 *
 *	A very simple I/O port RAM drive. Note that the CP/M parity and other
 *	goodies are all done in software. The hardware is really simple.
 *
 *	Three 74LS161 counters. They provide the bits A0-A21 (and 2 spare).
 *	A write to them writes to the low 8bits whilst the previous bits
 *	ripple upwards. A0-A18 fed 512K of DRAM, A19-A21 compare with the
 *	board switches to see which board is selected. Simples!
 *
 *	We only model one board
 */

static uint8_t mdrive[512 * 1024];
static uint32_t mdptr;

static uint8_t mdrive_read(uint8_t addr)
{
	uint8_t r = 0xff;
	addr &= 1;
	if (addr == 0) {
		if (trace & TRACE_MDRIVE)
			fprintf(stderr, "mdrive read ptr = %06X, ", mdptr);
		if (mdptr < sizeof(mdrive))
			r = mdrive[mdptr];
		if (trace & TRACE_MDRIVE)
			fprintf(stderr, "data %02X\n", r);
		mdptr++;
	}
	/* The pointer is 22 bits long */
	mdptr &= 0x3FFFFF;
	return r;
}

static void mdrive_write(uint8_t addr, uint8_t val)
{
	addr &= 1;
	if (addr == 0) {
		if (trace & TRACE_MDRIVE)
			fprintf(stderr, "mdrive write ptr = %06X, data %02X\n",
				mdptr, val);
		if (mdptr < sizeof(mdrive))
			mdrive[mdptr] = val;
		mdptr++;
	} else {
		mdptr <<= 8;
		mdptr |= val;
		if (trace & TRACE_MDRIVE)
			fprintf(stderr, "mdptr set: %06X\n", mdptr);
	}
	mdptr &= 0x3FFFFF;
}

/*
 *	ALT256. We need to write some rendering support and SDL code for
 *	this to be any use!
 */
static uint8_t alt256[256 * 256];
static uint8_t alt256_x, alt256_y;
static uint8_t alt256_wipe;
static uint8_t alt256_wval;
static uint8_t alt256_clock;

static uint8_t alt256_read(uint8_t addr)
{
	uint8_t r = 0xff;
	addr &= 3;
	if (addr == 0) {
		r = alt256_wipe;
		/* This is a guestimate of vblank */
		if (alt256_clock < 2 || alt256_clock > 16)
			r |= 2;
		r |= 0xFC;
	}
	return r;
}

static void alt256_write(uint8_t addr, uint8_t val)
{
	addr &= 3;
	switch(addr) {
	case 0:
		if (alt256_wipe)
			return;
		/* We should check this is 3.4us or more after the last ? */
		alt256[256 * alt256_y + alt256_x] = (val & 1) ? 0xFF : 0x00;
	case 1:
		alt256_x = val;
		break;
	case 2:
		alt256_y = val;
		break;
	case 3:
		alt256_wipe = 1;
		alt256_wval = val & 1;
		break;
	}
}

/* Called every 5ms */
static void alt256_tick(void)
{
	alt256_clock++;
	if (alt256_clock == 20) {	/* Frame end */
		if (alt256_wipe) {
			memset(alt256, (alt256_wval & 1) ? 0xFF : 0x00, sizeof(alt256));
			alt256_wipe = 0;
		}
	}
}

/*
 *	RTC: Emulated as a read only device with no interrupt for now
 *
 *	Based on the Dual systems Clk-24 with the write protect jumper set
 *	and in 24 hour mode with the interrupt wired via 8085 RST55 at once
 *	per second.
 */

static uint8_t msmctrl;
static uint8_t msmintr;
static uint8_t msmien;
static uint8_t msmhold;

static uint8_t msm5832_read(uint8_t addr)
{
	static time_t t;
	struct tm *tm;
	uint8_t r = 0xFF;

	addr &= 1;
	if (addr == 0) {
		if (msmhold == 0) {
			time(&t);
			if (!(msmctrl & 0x80)) {
				if (trace & TRACE_RTC)
					fprintf(stderr, "[msm5832 hold]\n");
				msmhold = 10;	/* 1/2 second */
			}
		}
		if (trace & TRACE_RTC)
			fprintf(stderr, "latched time = %s", ctime(&t));
		tm = gmtime(&t);
		if (tm == NULL) {
			fprintf(stderr, "localtime error.\n");
			exit(1);
		}
		switch(msmctrl & 0x0F) {
			case 0:
				r = tm->tm_sec % 10;
				break;
			case 1:
				r = tm->tm_sec / 10;
				break;
			case 2:
				r = tm->tm_min % 10;
				break;
			case 3:
				r = tm->tm_min / 10;
				break;
			case 4:
				r = tm->tm_hour % 10;
				break;
			case 5:
				r = tm->tm_hour / 10 | 8;
				if (tm->tm_hour >= 12)
					r |= 4;
				break;
			case 6:
				r = tm->tm_wday;
				break;
			case 7:
				r = tm->tm_mday % 10;
				break;
			case 8:
				/* FIXME: leap year flag */
				r = tm->tm_mday / 10;
				break;
			case 9:
				r = tm->tm_mon % 10;
				break;
			case 10:
				r = tm->tm_mon / 10;
				break;
			case 11:
				r = tm->tm_year % 10;
				break;
			case 12:
				r = (tm->tm_year / 10) % 10;
				break;
		}
		if (trace & TRACE_RTC)
			fprintf(stderr, "MSM read %d = %d\n",
				msmctrl, r);
	}
	return r;
}

static void msm5832_write(uint8_t addr, uint8_t val)
{
	switch(addr & 1) {
		case 0:
			break;
		case 1:
			if (msmctrl == 0x8F)
				msmien = 1;
			if (msmctrl == 0x8E)
				i8085_clear_int(INT_RST55);
			msmctrl = val & 0x8F;
			if (val & 0xC0)
				msmhold = 0;
			if (trace & TRACE_RTC)
				fprintf(stderr, "MSM selected = %d\n", msmctrl & 0x0F);
			break;
	}
}

static void msm5832_tick(void)
{
	if (msmhold)
		msmhold--;
	msmintr++;
	if (msmintr == 20) {
		msmintr = 0;
		if (msmien)
			i8085_set_int(INT_RST55);
	}
}

/*
 *	Simple timer: modelled on the MITS VI/RTC but only the RTC side. We
 *	ignore emulation of the 880-VI interrupt structure or indeed anything
 *	but 8085 interrupt lines.
 */

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
	if (trace & TRACE_BANK)
		fprintf(stderr, "Bank select %02X\n", bank);
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
		fprintf(stderr, "PC = %04X\n", i8085_read_reg16(PC));
		banknum = 8;
		break;
	}
	if (!(bank & bankmap))
		banknum = 8;
}

uint8_t i8085_inport(uint8_t addr)
{
	if (trace & TRACE_IO)
		fprintf(stderr, "read %02x\n", addr);
	if (addr >= 0x00 && addr <= 0x01)
		return acia_read(addr & 1);
	if (addr >= 0x10 && addr <= 0x17)
		return my_ide_read(addr & 7);
	if (addr >= 0x18 && addr <= 0x1f)
		return fdc_read(addr);
	if (addr >= 0x20 && addr <= 0x2F)
		return i8237_read(addr);
	if (addr >= 0xC6 && addr <= 0xC7)
		return mdrive_read(addr);
	if (addr >= 0xE0 && addr <= 0xE3)
		return alt256_read(addr);
	if (addr >= 0xF0 && addr <= 0xF1)
		return msm5832_read(addr);
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
	else if (addr >= 0x18 && addr <= 0x1F)
		fdc_write(addr, val);
	else if (addr >= 0x20 && addr <= 0x2F)
		i8237_write(addr, val);
	else if (addr == 0x40)
		bank_write(val);
	else if (addr >= 0xC6 && addr <= 0xC7)
		mdrive_write(addr, val);
	else if (addr >= 0xE0 && addr <= 0xE3)
		alt256_write(addr, val);
	else if (addr >= 0xF0 && addr <= 0xF1)
		msm5832_write(addr, val);
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
	done = 1;
}

static void exit_cleanup(void)
{
	tcsetattr(0, TCSADRAIN, &saved_term);
}

static void usage(void)
{
	fprintf(stderr, "v85: [-b banks] [-f] [-d debug]\n");
	exit(EXIT_FAILURE);
}

int main(int argc, char *argv[])
{
	static struct timespec tc;
	int opt;
	int fd;
	int cycles;

	while ((opt = getopt(argc, argv, "b:d:f")) != -1) {
		switch (opt) {
		case 'b':
			bankmap = atoi(optarg) | 1;
			break;
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
	if (read(fd, rom, 512) < 8) {
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


	fdc = fdc_new();

	lib765_register_error_function(fdc_log);

	if (access("drivea.dsk", 0) == 0) {
		drive_a = fd_newdsk();
		fd_settype(drive_a, FD_525);
		fd_setheads(drive_a, 2);
		fd_setcyls(drive_a, 80);
		fdd_setfilename(drive_a, "drivea.dsk");
	} else
		drive_a = fd_new();

	if (access("driveb.dsk", 0) == 0) {
		drive_b = fd_newdsk();
		fd_settype(drive_a, FD_525);
		fd_setheads(drive_a, 2);
		fd_setcyls(drive_a, 80);
		fdd_setfilename(drive_a, "driveb.dsk");
	} else
		drive_b = fd_new();

	drive_c = fd_new();

	fdc_reset(fdc);
	fdc_setisr(fdc, NULL);

	fdc_setdrive(fdc, 0, drive_a);
	fdc_setdrive(fdc, 1, drive_b);
	fdc_setdrive(fdc, 2, drive_c);
	fdc_setdrive(fdc, 3, drive_c);

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

	/* We run 150 cycles per I/O check, do that 200 times then poll the
	   slow stuff and nap for 5ms. */

	cycles = tstate_steps;

	if (trace & TRACE_CPU)
		i8085_log = stderr;

	while (!done) {
		int i;
		/* 30000 T states */
		for (i = 0; i < 200; i++) {
			cycles = tstate_steps + i8237_execute(cycles);
			if (cycles >= 0)
				cycles = tstate_steps + i8085_exec(cycles);
			acia_timer();
		}
		/* Do 5ms of I/O and delays */
		if (!fast)
			nanosleep(&tc, NULL);
		timer_tick();
		msm5832_tick();
		alt256_tick();
	}
	fd_eject(drive_a);
	fd_eject(drive_b);
	fd_eject(drive_c);
	fdc_destroy(&fdc);
	fd_destroy(&drive_a);
	fd_destroy(&drive_b);
	fd_destroy(&drive_c);
	exit(0);
}
