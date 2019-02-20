.sect .text
.sect .rom
.sect .data
.sect .bss

.sect .text


start:
	di
	! Set up the ACIA
	mvi a,0x16
	out 0
	!
	!	As early as possible display something
	!
	lxi h,signon
	mvi a,'V'
	out 1
	lxi sp,0
	call print
	!
	!	Now copy the scanner up to F000 (common space)
	!
	lhld scanit
	shld 0xf000
	lhld scanit+2
	shld 0xf002
	lhld scanit+4
	shld 0xf004
	lhld scanit+6
	shld 0xf006
	lhld scanit+8
	shld 0xf008

	lxi h,memcheck
	call print

	mvi a,1
	lxi d,0
	lxi h,0x8000
bankcheck:
	mvi c,0xAA
	mov b,a
	call 0xf000
	mov a,c
	cpi 0xAA
	jnz bankabsent
	mvi c,0x55
	mov a,b
	call 0xf000
	mov a,c
	cpi 0x55
	jz bankfound
bankabsent:
	mvi a,' '
	jmp banknext
bankfound:
	inr e
	mov a,d
	adi 48
banknext:
	call pchar
	inr d
	mov a,b
	add a
	jnz bankcheck

	lxi h,bankdone
	call print

	! We have E * 48 + 16K of RAM
	! Might be nice to print it out for the user

	mvi a,0xe0
	out 0x16 ! head & dev,a
	call waitready
	mvi a,1
	out 0x11 	! feature
	mvi a,0xEF	! Set features - 8bit mode
	out 0x17 	! command
	call waitready
	xra a
	out 0x13	! LBA 0-2
	out 0x14
	out 0x15
	! We set LBA3 to E0 already
	inr a
	out 0x12	! Count
	mvi a,0x20	! Read Sector
	out 0x17 	! command
	lxi h,0xFE00	! buffer target
	call waitdrq
	mvi b,0
sector:
	in 0x10 	! Data
	mov m,a
	inx h
	in 0x10
	mov m,a
	inx h
	dcr b
	jnz sector
	call waitready
	lda 0xFE00
	cpi 0x85
	jnz badload
	lda 0xFE01
	cpi 0x80
	jz 0xFE02
badload:
	lxi h,badboot
	call print
	hlt

waitdrq:
	in 0x17		! Status
	ani 0x08	! DRQ
	jz waitdrq
	ret

waitready:
	in 0x17
	ani 0x40
	jz waitready
	ret

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
pspace:
	mvi a,32
pchar:
	push psw
pcharw:
	in 0
	ani 2
	jz pcharw
	pop psw
	out 1
	ret

.sect .rom
signon:
	.ascii "85 ROM BIOS 0.1"
	.data1 13,10,0
memcheck:
	.asciz "Banks present: "
bankdone:
	.data1 13,10,13,10
	.asciz "Loading ..."
badboot:
	.ascii "Not a valid boot block"
	.data1 13,10,0
scanit:				! A tiny probe routine (9 bytes)
	out 0x40
	mov m,c
	mov c,m
	xra a
	out 0x40
	ret
