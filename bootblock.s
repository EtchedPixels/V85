!
!	A simple bootblock to run at 0xFF00
!
.sect .text
.sect .rom
.sect .data
.sect .bss
.sect .text

	.data2 0x8085

start:
	lxi h,helloworld
	call print
	hlt

print:
	in 0
	ani 2
	jz print
	mov a,m
	ora a
	rz
	out 1
	inx h
	jmp print

.sect .data

helloworld:
	.asciz 'Hello World'
