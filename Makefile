
CFLAGS = -Wall -pedantic -O2

all:	v85 makedisk v85.rom bootblock

v85:	v85.o intel_8085_emulator.o ide.o
	cc -g3 $^ -o v85

ack2rom: ack2rom.c

v85.rom: ack2rom rom.s
	ack -mcpm -c rom.s
	/opt/ackcc/lib/ack/em_led -b0:0x0000 rom.o -o rom.bin
	./ack2rom <rom.bin >v85.rom

bootblock: ack2rom bootblock.s
	ack -mcpm -c bootblock.s
	/opt/ackcc/lib/ack/em_led -b0:0xFF00 bootblock.o -o bootblock.bin
	./ack2rom <bootblock.bin >bootblock

makedisk: makedisk.o ide.o
	cc -O2 -o makedisk makedisk.o ide.o

clean:
	rm -f *.o *~ v85 makedisk v85.rom rom.bin ack2rom
	rm -f bootblock.bin bootblock

