.sect .text
.sect .rom
.sect .data
.sect .bss

.sect .text


start:
	di
	lxi h,signon
loop:
	in 0
	ani 2
	jz loop
	mov a,m
	ora a
	jz setup
	out 1
	inx h
	jmp loop
setup:
	hlt

.sect .rom
signon:
	.asciz "Hello World"
