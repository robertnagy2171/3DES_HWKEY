	title	"Microchip PIC 12F509 based HW Key for triple DES encryption/decryption"
	subtitl	"Definitions"
	list	c=135, p=12f509, r=DEC, f=INHX8M
;
;-----------------------------------------------------------------------
;			HWKEY.ASM
;----------------------------------------------------------------

#include "p12f509.inc"		; Standard equates & Macros

;----------------------------------------------------------------
;	Sort this out eventually.  Currently TxByte and
;	RxByte have hard coded instructions for Bit 6.
;	***Check those routines if you change this****

TXbit	equ	3
TXmask	equ	0F7h
RXbit	equ	2

;----------------------------------------------------------------
;	Some Macros to help readability

jz	Macro	label		; jump if zero
	btfsc	status,z
	goto	label
	endm

jnz	Macro	label		; jump if not zero
	btfss	status,z
	goto	label
	endm

PortW	Macro	PortX
	tris	PortX
	endm

map	Macro	r1,b1,r2,b2	; Map a bit in a byte to another one
	btfsc	r1,b1
	bsf	r2,b2
	endm

;	Used in the DES routines to do the key expansion. It maps specified bits of various bytes into bits 5-0 of the target byte
BuildEkey Macro	r,r1,b1,r2,b2,r3,b3,r4,b4,r5,b5,r6,b6
	clrf	r ; clear the target byte
	map	r1,b1,r,5
	map	r2,b2,r,4
	map	r3,b3,r,3
	map	r4,b4,r,2
	map	r5,b5,r,1
	map	r6,b6,r,0
	endm

;	Used in the DES routines to store four bits of an S-Box lookup (lo nibble) into the specified byte(s):bit(s)
StoreLo	Macro	r1,b1,r2,b2,r3,b3,r4,b4
	movwf	Etemp
	map	Etemp,3,r1,b1
	map	Etemp,2,r2,b2
	map	Etemp,1,r3,b3
	map	Etemp,0,r4,b4
	endm

;	Used in the DES routines to store four bits of an S-Box lookup (hi nibble) into the specified bytes:bits
StoreHi	Macro	r1,b1,r2,b2,r3,b3,r4,b4
	movwf	Etemp
	map	Etemp,7,r1,b1
	map	Etemp,6,r2,b2
	map	Etemp,5,r3,b3
	map	Etemp,4,r4,b4
	endm

;	Convert data into a 'retlw' type table (8 bytes)
Rdata	Macro	k1,k2,k3,k4,k5,k6,k7,k8
	retlw	k1
	retlw	k2
	retlw	k3
	retlw	k4
	retlw	k5
	retlw	k6
	retlw	k7
	retlw	k8
	endm

;----------------------------------------------------------------
;	File register usage

Etemp	equ	TMR0	; Used during 'encrypt' process

crypt	equ	07h	; crypto reg
DE_BIT	equ	0 ; 0. bit - control flag (0 encryption, 1 decription)
KEY_BIT	equ	1 ; 1. bit - key selector (0 key0, 1 key1)
TF_BIT	equ	2 ; format mode during encryption/decryption (0 Bin, 1 ASCII)

ek1	equ	08h	; +--------+
ek2	equ	09h	;	   |
ek3	equ	0Ah	; expanded |
ek4	equ	0Bh	; (working)|
ek5	equ	0Ch	;   DES    |
ek6	equ	0Dh	;   key    |
ek7	equ	0Eh	;	   |
ek8	equ	0Fh	; +--------+

Dcount	equ	09h	; Used in delay routine when Rx data
Uartdata	equ	0Ah	; Used to hold UART data
RxCount	equ	0Bh	; Used when rx bytes
Icount	equ	0Ch	; Used to determine which Instruction

r18	equ	10h	; +--------+
r19	equ	11h	;  data    |
r1A	equ	12h	;          |
r1B	equ	13h	;          |
r1C	equ	14h	;          |
r1D	equ	15h	;          |
r1E	equ	16h	;          |
r1F	equ	17h	; +--------+

r14	equ	18h
r15	equ	19h
r16	equ	1Ah
r17	equ	1Bh

bf1	equ	10h	; 1 byte for command
bf2	equ	11h	; ---------+
bf3	equ	12h	; 12 bytes |
bf4	equ	13h	;    for   |
bf5	equ	14h	;  params  |
bf6	equ	15h	;          |
bf7	equ	16h	;          |
bf8	equ	17h	;          |
bf9	equ	18h	;          |
bfA	equ	19h	;          |
bfB	equ	1Ah	;          |
bfC	equ	1Bh	;          |
bfD	equ	1Ch	; ---------+

mode	equ	1Dh	; Used to hold working mode in ascii
F_BIT	equ	0 ; 0. bit - data format (0 Bin, 1 ASCII)
TDS_BIT	equ	1 ; 1. bit - type of encryption/decryption method (0 DES, 1 Triple DES)

Rcount	equ	1Eh	; Round counter
Ubcount	equ	1Fh	; Used as bit counter in UART routine

k1	equ	30h	; +--------+
k2	equ	31h	;          |
k3	equ	32h	;          |
k4	equ	33h	;  DES Key |
k5	equ	34h	;          |
k6	equ	35h	;          |
k7	equ	36h	; +--------+

	subtitl	"General Routines"
	page
	org 0
;----------------------------------------------------------------
;	Power up/Reset starting point

reset	call	IOres		; reset of the port
	goto	reset2		; contuniue reset on the first page

;----------------------------------------------------------------
;	Subrutins

LookUp	movwf	pcl		; Go to it

;-----------------------------------------------------------------
; TxByte - send a byte to the PC

TxByte	movwf	Uartdata	; Put byte in Uartdata
	movlw	TXmask
	portw	GPIO		; Set o/p bit
	bsf	GPIO,TXbit	; Start bit
	bsf	Ubcount,3	; set to 8
Tx_lp	call	DelayXX		; Bit delay
	rrf	Uartdata,f	; get MSB to bit 6
	clrw			; Prepare to send 1
	btfss	status,c	; Skip is really 1
	movlw	0FFh		; Send 0
	movwf	GPIO		; Send next bit
	decfsz	Ubcount,f
	goto	Tx_lp
	call	DelayXX
	bcf	GPIO,TXbit	; Stop bit
	call	DelayXX
IOres	movlw	0FFh		; Port back to i/p
	portw	GPIO
	retlw	0

;----------------------------------------------------------------
; RxByte - receive a byte from the PC, the result is stored in Uartdata

RxByte	btfss	GPIO,RXbit
	goto	RxByte
	movlw	45
	call	DelayW
	bsf	Ubcount,3	; set to 8
RxLoop	bcf	status,c
	btfss	GPIO,RXbit
	bsf	status,c
	rrf	Uartdata,f
	call	DelayXX
	decfsz	Ubcount,f
	goto	RxLoop
	movlw	15
	goto	DelayW		; delay and retlw

;-----------------------------------------------------------------------
; Delay routines

DelayXX	movlw	30		; 30 loops
DelayW	movwf	Dcount		; Set delay counter
DelayLp	decfsz	Dcount,f
	goto	DelayLp
	retlw	0

;-----------------------------------------------------------------------
;	Table of Instructions

InsTab	retlw	049h	; 'I' command, warm reset
	retlw	056h	; 'V' command, version of the software
	retlw	04Dh	; 'M' command, mode selection
	retlw	045h	; 'E' command, DES encryption
	retlw	044h	; 'D' command, DES decryption
	retlw	0Dh	; NOP

Jtable	goto	Reset
	goto	Ins_V
	goto	Ins_M
	goto	Ins_E
	goto	Ins_D
	goto	Complete

;	We've recognised an instruction...
GotIns	movlw	Jtable-1	; Base address
	addwf	Icount,w	; W = Base address + Icount
	goto	LookUp		; jump

;-----------------------------------------------------------------------
; GetShf
;	Look up and return the number of shifts required for
;	this round of the algorithm (table indexed in reverse)

GetShf	movf	Rcount,w	; r=Rcount (round count)
	addwf	pcl,f		; index into shift table

	Rdata	1,2,2,2,2,2,2,1
	Rdata	2,2,2,2,2,2,1,1	

rbf1	rrf	bf1,f
rbf2	rrf	bf2,f
rbf3	rrf	bf3,f
rbf4	rrf	bf4,f
rbf5	rrf	bf5,f
rbf6	rrf	bf6,f
rbf7	rrf	bf7,f
rbf8	rrf	bf8,f
rbf9	rrf	bf9,f
rbfA	rrf	bfA,f
rbfB	rrf	bfB,f
	retlw	0

;-----------------------------------------------------------------------
; Encryption
;	Using the key in k1-k7 encrypt the data in 18-1F
;	(Working key in r28-2F   - data working area 14-17)

; Initial permutation

DEScrypt	movlw	r1f		; initial permutation
	movwf	fsr
	movlw	08h		; 8 bytes
	movwf	RCount

PD_lp	rrf	indf,f
	rlf	ek4,f
	rrf	indf,f
	rlf	ek8,f
	rrf	indf,f
	rlf	ek3,f
	rrf	indf,f
	rlf	ek7,f
	rrf	indf,f
	rlf	ek2,f
	rrf	indf,f
	rlf	ek6,f
	rrf	indf,f
	rlf	ek1,f
	rrf	indf,f
	rlf	ek5,f

	decf	fsr,f
	decfsz	RCount,f
	goto	PD_lp

	bsf	status,pa0	; MPData on page1
	call	MPData		; set back to page0

	bsf	fsr,5	; switch to bank1

	movlw	0F0h	; Key '0'
	movwf	k1
	movlw	0CCh
	movwf	k2
	movlw	0AAh
	movwf	k3
	movlw	0F5h
	movwf	k4
	movlw	056h
	movwf	k5
	movlw	067h
	movwf	k6
	movlw	08Fh
	movwf	k7

	btfss	crypt,KEY_BIT	; skip if key1
	goto	Encrypt

	movlw	0h	; Key '1'
	movwf	k1
	movlw	0h
	movwf	k2
	movlw	0h
	movwf	k3
	movlw	0h
	movwf	k4
	movlw	0h
	movwf	k5
	movlw	0h
	movwf	k6
	movlw	0h
	movwf	k7

Encrypt	bcf	fsr,5		; switch back to bank0
	movlw	10h		; 16 rounds (15->0 encryption) 
	movwf	Rcount

Enc_lp	decf	Rcount,f
	bsf	status,pa0	; Enc_fin on page1
	btfsc	Rcount,7	; Exit when Rcount goes -ve
	goto	Enc_fin
	bcf	status,pa0	; back to page0

;	We have 56 bits of key in k1....k7 rotate left the
;	two 28 bits halves (split is in the middle of k4)
;	We do the shift all-at-once and then deal with the
;	carry bits

	call	GetShf		; Get the shift count
	movwf	Etemp

	btfsc	crypt,DE_BIT	; skip if encrypt
	goto	Jshift

Ishift	bsf	fsr,5		; switch to bank1 for k regs
Ish_lp	bcf	status,c	; Clear carry
	rlf	k7,f		; Shift it all left
	rlf	k6,f
	rlf	k5,f
	rlf	k4,f
	rlf	k3,f		; shift k1-23 left
	rlf	k2,f
	rlf	k1,f
	btfsc	k4,4		; if this is set...
	bsf	k7,0		; ...rotate
	bcf	k4,4		; Clear (in case carry clear)
	btfsc	status,c
	bsf	k4,4		; Rotate carry to k4:4
	decfsz	Etemp,f
	goto	Ish_lp

Bekey	bsf	status,pa0
	goto	Bekey2

Jshift	addwf	Rcount,w	;
	bsf	fsr,5		; switch to bank1 for k regs
				; trick for detection the case when sum of rcount and w is at least 16
	btfsc	status,dc
	goto	Bekey

Jsh_lp	bcf	status,c	; Clear carry
	rrf	k1,f		; Shift it all right
	rrf	k2,f
	rrf	k3,f
	rrf	k4,f
	rrf	k5,f		; shift k1-23 right
	rrf	k6,f
	rrf	k7,f
	btfsc	k4,3		; if this is set...
	bsf	k1,7		; ...rotate
	bcf	k4,3		; Clear (in case carry clear)
	btfsc	status,c
	bsf	k4,3		; Rotate carry to k4:3
	decfsz	Etemp,f
	goto	Jsh_lp
	goto	Bekey

;	We've done the shift, now we need to do the Permutation
;	to produce the 48-bit key according to table PC-2.
;	To simplify the following the code we permute the key
;	into the lower 6 bits of 8 bytes. (ek<n> - expanded key n)

;-----------------------------------------------------------------------
;	This is the first S-Box table.  Each is used twice; each
;	entry is actually 4-bits in size so the first 8 lookups
;	extract the low nibble - the remaining eight the hi nibble.
;
;	code locaton: 0C0h-0FFh

	org	0C0h
	list	r=HEX

S1_Box	Rdata	02E,0E0,0C4,0BF,04D,027,011,0C4
	Rdata	072,04E,0AF,072,0BB,0DD,068,011
	Rdata	083,05A,05A,006,036,0FC,0FC,0AB
	Rdata	0D5,039,009,095,0E0,083,097,068
	Rdata	044,0BF,021,08C,01E,0C8,0B8,072
	Rdata	0AD,014,0D6,0E9,072,021,08B,0D7
	Rdata	0FF,065,09C,0FB,0C9,003,057,09E
	Rdata	063,0AA,03A,040,005,056,0E0,03D
	list	r=DEC

;-----------------------------------------------------------------------
; A part of Encrypt.

Enc_lp_pre	movf	r14,w
	movwf	r1C
	movf	r15,w
	movwf	r1D
	movf	r16,w
	movwf	r1E
	movf	r17,w
	movwf	r1F

	goto	Enc_lp

;----------------------------------------------------------------
;	Start

reset2	movlw	30h		; default mode 0
	movwf	mode

	clrf	Ubcount		; clear bit counter in UART routine

;----------------------------------------------------------------
;	This is where we send a command completion response
;	It is Msg_OK "OK\r\n"
	goto Complete
	
;	Do some initialisation and then wait for a command from the PC.
;	This is read into the buffer


;-----------------------------------------------------------------------
; Read 13 chars into buffer at bf1-bfD
; AABBBBBBBBAAA	(A=ascii, B=binary)	if the mode Binary
; AAAAAAAAAAAAA	(A=ascii, B=binary)	if the mode ASCII
; In case of ASCII, 0Dh can terminate the reading process.
; It is waiting for 0Dh after the 13th character

Idle
ReadBuf	bsf	crypt,TF_BIT	; default mode is ascii
	call	RxByte		; read the first, skip if 0Dh
	movf	Uartdata,w
	movwf	bf1
	xorlw	0Dh
	jz	RB_end
	btfsc	mode,F_BIT	; skip if binary mode is selected
	goto	RB_2nd

	movf	bf1,w
	xorlw	44h		; 'D' command?
	btfsc	status,z
	bcf	crypt,TF_BIT	; yes, accept binary mode
	xorlw	01h		; 'E' command?
	btfsc	status,z
	bcf	crypt,TF_BIT	; yes, accept binary mode

RB_2nd	call	RxByte		; read the second, skip if 0Dh
	movf	Uartdata,w
	movwf	bf2
	xorlw	0Dh
	jz	RB_end

	movlw	bf3		; beggining of command data in the buffer
	movwf	fsr
	movlw	08h		; 8 bytes to receive
	movwf	RxCount
RB_lp	call	RxByte
	movf	Uartdata,w
	movwf	indf
	btfss	crypt,TF_BIT	; don't check 0Dh in binary mode during encryption/decription commands
	goto	RB_next
	xorlw	0Dh
	jz	RB_end
RB_next	incf	fsr,f
	decfsz	RxCount,f
	goto	RB_lp

	movlw	03h		; 3 bytes to receive
	movwf	RxCount
RB_lp2	call	RxByte
	movf	Uartdata,w
	movwf	indf
	xorlw	0Dh
	jz	RB_end
	incf	fsr,f
	decfsz	RxCount,f
	goto	RB_lp2

RB_CR	call	RxByte		; read for CR
	movf	Uartdata,w
	xorlw	0Dh
	jnz	RB_CR
RB_end	movlw	0Ah		; LF
	call	TxByte

;	We've received a command, process it.
	movlw	6		; there are 6 valid instructions
	movwf	Icount

FndCmd	movlw	InsTab-1	; Table base
	addwf	Icount,w	; Index into table
	call	LookUp		; Load value
	xorwf	bf1,w		; Compare with Instruction type (first char)
	jz	GotIns		; Skip if the same
	decfsz	Icount,f	; Loop if more
	goto	FndCmd
	goto	Idle		; do not write OK, invalid command

;----------------------------------------------------------------
; 'I' initialisation - warm reset
;
; usage (< input, > output):
; < I\r
; > \n
; > OK\r\n

; Ins_I	goto	reset ; it is implemented in the Jtable for saving program memory

;----------------------------------------------------------------
; 'V' version info - max 2 chars because of memory limitation
;
; usage (< input, > output):
; < V\r
; > \n
; > version\r\n
; > OK\r\n

Ins_V
	movlw	41h		; A1
	movwf	bf3
	movlw	31h
	movwf	bf4
	movlw	02h
	movwf	Rcount		; length of message
	goto	WriteBuf

;----------------------------------------------------------------
; 'Mx' set mode
;
; It sets the mode of encryption/decryption where x means:
; '0' - DES, binary
; '1' - DES, ASCII
; '2' - 3DES, binary 
; '3' - 3DES, ASCII
;
; If x was not provided, it shows the current value of mode
;
; usage (< input, > output):
; < M\r
; > \n
; > x\r\n
; > OK\r\n
;
; < Mx\r
; > \n
; > OK\r\n
;

Ins_M	movlw	0Dh	; CR
	xorwf	bf2,w	; x is not provided
	jz	M_nox
	movlw	03h
	andwf	bf2,w
	movwf	mode
	movlw	30h
	addwf	mode,f
	goto	Complete

M_nox	movf	mode,w
	movwf	bf1
	movlw	01h
	movwf	Rcount		; length of message
;----------------------------------------------------------------
; WriteBuf
;	Send out the content of the buffer. The length is in Rcount.

WriteBuf	movlw	bf1		; beggining of the buffer 
	movwf	fsr

WB_lp	movf	indf,w
	call	TxByte
	incf	fsr,f
	decfsz	RCount,f
	goto	WB_lp

	movlw	0Dh		; CR
	call	TxByte
	movlw	0Ah		; LF
	call	TxByte

Complete	movlw	4Fh		; 'O'
	call	TxByte
	movlw	4Bh		; 'K'
	call	TxByte
	movlw	0Dh		; CR
	call	TxByte
	movlw	0Ah		; LF
	call	TxByte
	goto	Idle

;----------------------------------------------------------------
; 'E' DES Encryption : Ek<data>
; 
; k is the ID of the key ('0' or '1') for encryption.
; The 56-bit keys are stored on 7 bytes in the program memory
;
; The sequence of triple DES encryption is the following:
; 1. encryption with key k
; 2. decryption with key other than k
; 3. encryption with key k
; 
; <data> is the 64-bit data to be encrypted.
; It is represented on 8 bytes in binary mode and 11 bytes (ascii chars) in ASCII mode
; In ASCII mode the 64 bits are split according to the following schema: 6 6 6 6 6 6 6 6 6 6 4. Tha last 4 bits shifted left 2 times (right padded with 0) 
; All 6-bit values are incremented with 32 for moving the value into 32-95 (20h - 5Fh) interval
; 
; usage (< input, > output):
; < Ek<data>\r
; > \n
; > <encrypted data>\r\n
; > OK\r\n
;
;----------------------------------------------------------------
; 'D' DES Decryption : Dk<encrypted data>
;
; k is the ID of the key ('0' or '1') for encryption.
; The 56-bit keys are stored on 7 bytes in the program memory
;
; The sequence of triple DES decryption is the following:
; 1. decryption with key k
; 2. encryption with key other than k
; 3. decryption with key k
;
; <encrypted data> is the 64-bit data to be decrypted.
; It is represented on 8 bytes in binary mode and 11 bytes (ascii chars) in ASCII mode
; In ASCII mode the 64 bits are split according to the following schema: 6 6 6 6 6 6 6 6 6 6 4. Tha last 4 bits shifted left 2 times (right padded with 0) 
; All 6-bit values are incremented with 32 for moving the value into 32-95 (20h - 5Fh) interval
; 
; usage (< input, > output):
; < Dk<encrypted data>\r
; > \n
; > <data>\r\n
; > OK\r\n

Ins_D	bsf	crypt,DE_BIT	; flag is set for decrypt
	goto	Ins_DE
Ins_E	bcf	crypt,DE_BIT	; flag is set for encrypt

Ins_DE	bcf	crypt,KEY_BIT
	btfsc	bf2,0
	bsf	crypt,KEY_BIT

	btfss	mode,F_BIT	; skip if ascii mode
	goto	B_MODE

A_MODE	movlw	bf3	; point at beginning of data
	movwf	fsr

	movlw	0Bh	; 11 bytes to read
	movwf	Icount

A_1	movlw	20h	; convert ASCII bytes to binary (decrease with 20h and mask it with the relevant bits)
	subwf	indf,f
	movlw	3Fh
	andwf	indf,f

	incf	fsr,f
	decfsz	Icount,f
	goto	A_1

; put them together in the right place (bf1 - bf8, so r18 - r1F for DES)
	bcf	status,c

	rlf	bf4,f
	rlf	bf4,f
	rlf	bf4,f
	rlf	bf3,f
	rlf	bf4,f
	rlf	bf3,w
	movwf	bf1

	rlf	bf6,f
	rlf	bf6,f
	rrf	bf5,f
	rrf	bf6,f
	rrf	bf5,w
	rrf	bf6,f
	iorwf	bf4,w
	movwf	bf2
	movf	bf6,w
	movwf	bf3

	rlf	bf8,f
	rlf	bf8,f
	rlf	bf8,f
	rlf	bf7,f
	rlf	bf8,f
	rlf	bf7,w
	movwf	bf4

	rlf	bfA,f
	rlf	bfA,f
	rrf	bf9,f
	rrf	bfA,f
	rrf	bf9,w
	rrf	bfA,f
	iorwf	bf8,w
	movwf	bf5
	movf	bfA,w
	movwf	bf6

	rlf	bfC,f
	rlf	bfC,f
	rlf	bfC,f
	rlf	bfB,f
	rlf	bfC,f
	rlf	bfB,w
	movwf	bf7

	rrf	bfD,f
	rrf	bfD,w
	andlw	0Fh
	iorwf	bfC,w
	movwf	bf8

; call (triple) DES encryption/decryption on data
	call	DEScrypt
	btfss	mode,TDS_BIT	; Triple DES?
	goto	DEScr1		; no
	comf	crypt,f	; swap key
	call	DEScrypt
	comf	crypt,f	; swap key
	call	DEScrypt

DEScr1	clrf	bf9
	bcf	status,c

	call	rbf1	; separate binary 6 bits for converting to ASCII
	call	rbf1

	call	rbf2
	call	rbf2

	call	rbf3
	call	rbf3

	call	rbf4
	call	rbf4

	call	rbf5
	call	rbf5

	call	rbf6
	call	rbf6

	call	rbf7
	call	rbf7

	call	rbf8
	call	rbf8

	call	rbf9
	call	rbf9

	call	rbfA
	call	rbfA

	call	rbfB
	call	rbfB

	movlw	bf1	; point at beginning of response
	movwf	fsr

	movlw	0Bh	; 11 bytes to read
	movwf	Icount

A_2	movlw	3Fh ; convert binary to ASCII (mask it with the relevant bits and increase with 20h)
	andwf	indf,f
	movlw	20h
	addwf	indf,f

	incf	fsr,f
	decfsz	Icount,f
	goto	A_2

	movlw	0Bh
	goto	AB_MODE

B_MODE	movf	bf3,w ; move data to the right place (r18 - r1F for DES)
	movwf	r18
	movf	bf4,w
	movwf	r19
	movf	bf5,w
	movwf	r1A
	movf	bf6,w
	movwf	r1B
	movf	bf7,w
	movwf	r1C
	movf	bf8,w
	movwf	r1D
	movf	bf9,w
	movwf	r1E
	movf	bfA,w
	movwf	r1F

; call (triple) DES encryption/decryption on data
	call	DEScrypt
	btfss	mode,TDS_BIT	; Triple DES?
	goto	DEScr2		; no
	comf	crypt,f	; swap key
	call	DEScrypt
	comf	crypt,f	; swap key
	call	DEScrypt

DEScr2	movlw	08h
AB_MODE	movwf	Rcount		; length of message
	goto	WriteBuf

;-----------------------------------------------------------------------
;	These are the S-Box tables.  Each is used twice; each
;	entry is actually 4-bits in size so the first 8 lookups
;	extract the low nibble - the remaining eight the hi nibble.
;
;	code locaton: 200h-2BFh

	org	200h
	list	r=HEX

S2_Box	Rdata	0CF,0A3,011,0FD,0A8,044,0FE,027
	Rdata	096,07F,02B,0C2,063,098,084,05E
	Rdata	009,06C,0D7,010,032,0D1,04D,0EA
	Rdata	0EC,006,070,0B9,055,03B,0BA,085
	Rdata	090,04D,0EE,038,0F7,02A,05B,0C1
	Rdata	02A,093,084,05F,0CD,0F4,031,0A2
	Rdata	075,0BB,008,0E6,04C,017,0A6,07C
	Rdata	019,060,0D3,005,0B2,08E,06F,0D9

S3_Box	Rdata	04A,0DD,0B0,007,029,0B0,0EE,079
	Rdata	0F6,043,003,094,08F,016,0D5,0AA
	Rdata	031,0E2,0CD,038,09C,055,077,0CE
	Rdata	05B,02C,0A4,0FB,062,08F,018,061
	Rdata	01D,061,046,0BA,0B4,0DD,0D9,080
	Rdata	0C8,016,03F,049,073,0A8,0E0,077
	Rdata	0AB,094,0F1,05F,062,00E,08C,0F3
	Rdata	005,0EB,05A,025,09E,032,027,0CC

S4_Box	Rdata	0D7,01D,02D,0F8,08E,0DB,043,085
	Rdata	060,0A6,0F6,03F,0B9,070,01A,043
	Rdata	0A1,0C4,092,057,038,062,0E5,0BC
	Rdata	05B,001,00C,0EA,0C4,09E,07F,029
	Rdata	07A,023,0B6,01F,049,0E0,010,076
	Rdata	09C,04A,0CB,0A1,0E7,08D,02D,0D8
	Rdata	00F,0F9,061,0C4,0A3,095,0DE,00B
	Rdata	0F5,03C,032,057,058,062,084,0BE

LookUp2	movwf	pcl		; Go to it

;	Reorganise the Left and Right halves
;	18-1B -> 1C-1F,  14-17 -> 18-1B 

Enc_fin	movf	r18,w
	movwf	r1C
	movf	r19,w
	movwf	r1D
	movf	r1A,w
	movwf	r1E
	movf	r1B,w
	movwf	r1F

	movf	r14,w
	movwf	r18
	movf	r15,w
	movwf	r19
	movf	r16,w
	movwf	r1A
	movf	r17,w
	movwf	r1B

InvPData	movlw	ek8		; final permutation
	movwf	fsr
	movlw	08h		; 8 bytes
	movwf	RCount

IPD_lp	rlf	r1C,f
	rlf	indf,f
	rlf	r18,f
	rlf	indf,f
	rlf	r1D,f
	rlf	indf,f
	rlf	r19,f
	rlf	indf,f
	rlf	r1E,f
	rlf	indf,f
	rlf	r1A,f
	rlf	indf,f
	rlf	r1F,f
	rlf	indf,f
	rlf	r1B,f
	rlf	indf,f

	decf	fsr,f
	decfsz	RCount,f
	goto	IPD_lp

MPdata	movf	ek8,w
	movwf	r1F
	movf	ek7,w
	movwf	r1E
	movf	ek6,w
	movwf	r1D
	movf	ek5,w
	movwf	r1C
	movf	ek4,w
	movwf	r1B
	movf	ek3,w
	movwf	r1A
	movf	ek2,w
	movwf	r19
	movf	ek1,w
	movwf	r18
	bcf	status,pa0	;back to page0
	retlw	0

Bekey2
	BuildEkey ek1, k2,2,k3,7,k2,5,k3,0,k1,7,k1,3

;	clrf	ek1
;	map	k2,2,ek1,5
;	map	k3,7,ek1,4
;	map	k2,5,ek1,3
;	map	k3,0,ek1,2
;	map	k1,7,ek1,1
;	map	k1,3,ek1,0

	BuildEkey ek2, k1,5,k4,4,k2,1,k1,2,k3,3,k2,6
	BuildEkey ek3, k3,1,k3,5,k2,4,k1,4,k4,6,k1,0
	BuildEkey ek4, k2,0,k1,1,k4,5,k3,4,k2,3,k1,6
	BuildEkey ek5, k6,7,k7,4,k4,1,k5,3,k6,1,k7,1
	BuildEkey ek6, k4,2,k5,0,k7,5,k6,3,k5,7,k6,0
	BuildEkey ek7, k6,4,k7,7,k5,1,k7,0,k5,6,k7,3
	BuildEkey ek8, k6,2,k6,6,k7,6,k5,4,k4,3,k4,0

	bcf	fsr,5	; switch back to bank0

;	We've now built the working key up in ek1-ek8.  Clear
;	the area where we will store the result of the round...

	clrf	r14
	clrf	r15
	clrf	r16
	clrf	r17

;	Now select data bits according to the Bit-selection
;	table E and build up six bits in Etemp from data in 1C-1F
;	We want r1F:0 r1C:7 r1C:6 r1C:5 r1C:4 r1C:3

	rrf	r1F,w		; Get LSB of r1F into carry
	rrf	r1C,w		; Get 1C:7-3 into W:6-2
	movwf	Etemp		; Put in Etemp (bottom 2bits are junk)
	rrf	Etemp,f
	rrf	Etemp,w		; W = six bits of data...
	xorwf	ek1,w		; XOR with key
	andlw	03Fh		; Mask to six bits
	iorlw	0C0h		; S1_Box & 255
	bcf	status,pa0	; back to page0
	call	LookUp		; Do Sbox lookup
	bsf	status,pa0	; to page1

;	Finally we have to go through Permutation (P)
;	to get the bits in the right position in the
;	result

	StoreLo	r15,7, r16,7, r16,1, r17,1

;	Now build the next six bits of data in Etemp.
;	We need: r1C:4 r1C:3 r1C:2 r1C:1 r1C:0 r1D:7

	rlf	r1D,w		; Get r1D:7 into carry..
	rlf	r1C,w		; ..and thence into W with other bits
	xorwf	ek2,w		; XOR with key...
	andlw	3Fh		; Mask off six bits
	call	LookUp2		; Do Sbox lookup

	StoreLo	r15,3, r17,4, r14,6, r16,6

;	Now build the next six bits of data in Etemp.
;	We need: r1C:0 r1D:7 r1D:6 r1D:5 r1D:4 r1D:3

	rrf	r1C,w		; Put r1C:0 into carry
	rrf	r1D,w		; w=r1C:0,r1D:7.........
	movwf	Etemp
	rrf	Etemp,f
	rrf	Etemp,w		; w=data
	xorwf	ek3,w		; XOR key and data
	andlw	3Fh		; mask out 6 bits
	iorlw	040h		; S3_Box & 255
	call	LookUp2		; Do S-box lookup

	StoreLo	r16,0, r15,0, r17,2, r14,2

;	Next 6 data bits: r1D:4 r1D:3 r1D:2 r1D:1 r1D:0 r1E:7

	rlf	r1E,w		; Get r1E:7 into carry..
	rlf	r1D,w		; ..and thence into w with the rest
	xorwf	ek4,w		; XOR key and data
	andlw	3Fh		; Mask to six bits
	iorlw	080h		; S4_Box & 255
	call	LookUp2		; Do s-box lookup

	StoreLo	r17,6, r16,4, r15,6, r14,7

;	Next 6 data bits: r1D:0 r1E:7 r1E:6 r1E:5 r1E:4 r1E:3

	rrf	r1D,w		; Get LSB of r1D into carry
	rrf	r1E,w		; Get 1E:7-3 into W:6-2
	movwf	Etemp		; Put in Etemp (bottom 2bits are junk)
	rrf	Etemp,f
	rrf	Etemp,w		; w=data bits
	xorwf	ek5,w		; XOR with key
	andlw	03Fh		; Mask to 6 bits
	iorlw	0C0h		; S1_Box & 255
	bcf	status,pa0	; back to page0
	call	LookUp		; Do Sbox lookup
	bsf	status,pa0	; to page1

	StoreHi	r14,0, r15,2, r17,7, r14,5

;	Next 6 data bits: r1E:4 r1E:3 r1E:2 r1E:1 r1E:0 r1F:7

	rlf	r1F,w		; Get r1F:7 into carry..
	rlf	r1E,w		; ..and thence into w with the rest
	xorwf	ek6,w		; XOR with key
	andlw	3Fh		; mask to 6 bits
	call	LookUp2		; Do s-box lookup

	StoreHi	r14,4, r17,3, r15,5, r16,5

;	Next 6 data bits: r1E:0 r1F:7 r1F:6 r1F:5 r1F:4 r1F:3

	rrf	r1E,w		; Put r1E:0 into carry
	rrf	r1F,w		; w=r1E:0,r1F:7.........
	movwf	Etemp
	rrf	Etemp,f
	rrf	Etemp,w
	xorwf	ek7,w		; XOR with key
	andlw	3Fh		; Mask to 6 bits
	iorlw	040h		; S3_Box & 255
	call	LookUp2		; Do s-box lookup

	StoreHi	r17,0, r15,4, r16,2, r14,1

;	Final 6 data bits: r1F:4 r1F:3 r1F:2 r1F:1 r1F:0 r1C:7

	rlf	r1C,w		; Get r1C:7 into carry..
	rlf	r1F,w		; ..and thence into w (and the rest)
	xorwf	ek8,w		; XOR with key
	andlw	3Fh		; mask to 6 bits
	iorlw	080h		; S4_Box & 255
	call	LookUp2		; Do s-box lookup

	StoreHi	r14,3, r17,5, r15,1, r16,3

;	We've now done an encryption round, let's
;	XOR with the other half of the data in 18-1B

	movf	r18,w
	xorwf	r14,f
	movf	r19,w
	xorwf	r15,f
	movf	r1A,w
	xorwf	r16,f
	movf	r1B,w
	xorwf	r17,f

;	Reorganise the Left and Right halves
;	1C-1F -> 18-1B,  14-17 -> 1C-1F 

	movf	r1C,w
	movwf	r18
	movf	r1D,w
	movwf	r19
	movf	r1E,w
	movwf	r1A
	movf	r1F,w
	movwf	r1B

	bcf	status,pa0	;back to page0
	goto	Enc_lp_pre	; Next round

	END
