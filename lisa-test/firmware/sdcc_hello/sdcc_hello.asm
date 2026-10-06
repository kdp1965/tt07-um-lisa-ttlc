;--------------------------------------------------------
; File Created by SDCC : free open source ISO C Compiler
; Version 4.5.2 #0 (Mac OS X ppc)
;--------------------------------------------------------
	.module sdcc_hello
	
	.optsdcc -mlisa

; default segment ordering in RAM for linker
	.area DATA
	.area OSEG (OVR,DATA)

;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _main
	.globl _printf
	.globl _TIMER1_CTRL
	.globl _PORTB
	.globl _UART_STATUS
	.globl _UART_RXTX
	.globl _putchar
;--------------------------------------------------------
; special function registers
;--------------------------------------------------------
	.area RSEG (ABS)
	.org 0x0000
_UART_RXTX	=	0x0210
_UART_STATUS	=	0x0211
_PORTB	=	0x0201
_TIMER1_CTRL	=	0x020c
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area DATA
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area INITIALIZED
;--------------------------------------------------------
; overlayable items in ram
;--------------------------------------------------------
;--------------------------------------------------------
; Stack segment in internal ram
;--------------------------------------------------------
	.area SSEG
__start__stack:
	.ds	1

;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area DABS (ABS)
;--------------------------------------------------------
; interrupt vector
;--------------------------------------------------------
	.area HOME (CODE)
__interrupt_vect:
	jal	__sdcc_gsinit_startup
	rets
	rets
	rets
	rets
	rets
	rets
	rets
	rets
	rets
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME (CODE)
	.area GSINIT (CODE)
	.area GSFINAL (CODE)
	.area GSINIT (CODE)
	.area GSINIT (CODE)
__sdcc_gsinit_startup::
	ldx	#0x007f
	xchg	sp
	amode	1
	jal	___sdcc_external_startup
	cpi	#0
	if	ne
	jal	__sdcc_program_startup
	ldi	#>l_DATA
	push	a
	ldi	#<l_DATA
	push	a
	ldx	#s_DATA
00001$:
	ldax	1(sp)
	or	2(sp)
	bz	00002$
	ldi	#0
	stax	0(ix)
	adx	#1
	dcx	1(sp)
	if	c
	dcx	2(sp)
	br	00001$
00002$:
	ldi	#>s_INITIALIZED
	push	a
	ldi	#<s_INITIALIZED
	push	a
	ldi	#>l_INITIALIZED
	stax	4(sp)
	ldi	#<l_INITIALIZED
	stax	3(sp)
	ldx	#s_INITIALIZER
00003$:
	ldax	3(sp)
	or	4(sp)
	bz	00004$
	call	ix
	adx	#1
	push	ix
	ldxx	3(sp)
	stax	0(ix)
	inx	3(sp)
	if	c
	inx	4(sp)
	pop	ix
	dcx	3(sp)
	if	c
	dcx	4(sp)
	br	00003$
00004$:
	ads	#4
	.area GSFINAL (CODE)
	jal	__sdcc_program_startup
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME (CODE)
	.area HOME (CODE)
__sdcc_program_startup:
	jal	_main
00001$:
	br	00001$
;	return from main will return to caller
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CODE (CODE)
;	sdcc_hello.c: 20: int putchar(int c)
;	---------------------------------
;	 Function putchar
;	---------------------------------
_putchar:
	ads	#-1
;	sdcc_hello.c: 22: while (!(UART_STATUS & UART_TX_EMPTY))
00101$:
	lda	_UART_STATUS
	stax	1(sp)
	andi	#0x02
	bz	00101$
;	sdcc_hello.c: 24: UART_RXTX = c;
	ldax	4(sp)
	sta	_UART_RXTX
;	sdcc_hello.c: 25: return c;
	ldax	4(sp)
	stax	2(sp)
	ldax	5(sp)
	stax	3(sp)
00104$:
;	sdcc_hello.c: 26: }
	ads	#1
	ret
;	sdcc_hello.c: 43: static void print_lisa(void)
;	---------------------------------
;	 Function print_lisa
;	---------------------------------
_print_lisa:
	sra
	ads	#-4
;	sdcc_hello.c: 46: putchar('\n');
	ldi	#0x00
	push	a
	ldi	#0x0a
	push	a
	ads	#-2
	jal	_putchar
	ads	#4
;	sdcc_hello.c: 47: for (p = owl; *p; p++)
	ldi	#<(_owl + 0)
	stax	3(sp)
	ldi	#>(_owl + 0)
	stax	4(sp)
00103$:
	ldxx	3(sp)
	txau
	btst	7
	bz	00120$
	ldax	0(ix)
	stax	1(sp)
	ldax	1(ix)
	stax	2(sp)
	br	00121$
00120$:
	txau
	andi	#0x7f
	addaxu
	txa
	addax
	call	ix
	adx	#1
	stax	1(sp)
	call	ix
	stax	2(sp)
00121$:
	ldax	1(sp)
	or	2(sp)
	bz	00101$
;	sdcc_hello.c: 48: printf("%s\n", *p);
	ldax	2(sp)
	push	a
	ldax	2(sp)
	push	a
	ldi	#>(___str_0 + 0)
	push	a
	ldi	#<(___str_0 + 0)
	push	a
	ads	#-2
	jal	_printf
	ads	#6
;	sdcc_hello.c: 47: for (p = owl; *p; p++)
	ldax	3(sp)
	ldc	#0
	adc	#0x02
	stax	3(sp)
	ldax	4(sp)
	adc	#0x00
	stax	4(sp)
	br	00103$
00101$:
;	sdcc_hello.c: 49: printf("\nHello from TT07 LISA, compiled by sdcc -mlisa\n");
	ldi	#>(___str_1 + 0)
	push	a
	ldi	#<(___str_1 + 0)
	push	a
	ads	#-2
	jal	_printf
	ads	#4
00105$:
;	sdcc_hello.c: 50: }
	ads	#4
	lra
	ret
;	sdcc_hello.c: 52: void main(void)
;	---------------------------------
;	 Function main
;	---------------------------------
_main:
	sra
	ads	#-8
;	sdcc_hello.c: 55: unsigned int sum = 0;
	ldi	#0x00
	stax	7(sp)
	stax	8(sp)
;	sdcc_hello.c: 56: unsigned char ticks = 0;
	ldi	#0x00
	stax	6(sp)
;	sdcc_hello.c: 59: PORTB = 0x4f;
	ldi	#0x4f
	sta	_PORTB
;	sdcc_hello.c: 60: printf("\nsdcc_hello ready: ? = banner, s = count and sum\n");
	ldi	#>(___str_2 + 0)
	push	a
	ldi	#<(___str_2 + 0)
	push	a
	ads	#-2
	jal	_printf
	ads	#4
	ldi	#0x00
	stax	4(sp)
	stax	5(sp)
00114$:
;	sdcc_hello.c: 63: count++;
	inx	4(sp)
	if	c
	inx.p	5(sp)
;	sdcc_hello.c: 64: if (TIMER1_CTRL & TIMER_ROLLOVER)
	lda	_TIMER1_CTRL
	stax	3(sp)
	andi	#0x80
	bz	00104$
;	sdcc_hello.c: 66: if (++ticks == 4)
	inx	6(sp)
	ldax	6(sp)
	cpi	#0x04
	bnz	00104$
;	sdcc_hello.c: 68: ticks = 0;
	ldi	#0x00
	stax	6(sp)
;	sdcc_hello.c: 69: PORTB ^= 0x08;
	lda	_PORTB
	push	a
	ldi	#0x08
	swap	1(sp)
	xor	1(sp)
	ads	#1
	sta	_PORTB
00104$:
;	sdcc_hello.c: 72: if (UART_STATUS & UART_RX_AVAIL)
	lda	_UART_STATUS
	stax	3(sp)
	andi	#0x01
	bz	00114$
;	sdcc_hello.c: 74: c = UART_RXTX;
	lda	_UART_RXTX
;	sdcc_hello.c: 75: putchar(c);
	stax	3(sp)
	stax	1(sp)
	ldi	#0x00
	stax	2(sp)
	push	a
	ldax	2(sp)
	push	a
	ads	#-2
	jal	_putchar
	ads	#4
;	sdcc_hello.c: 76: sum += c;
	ldax	7(sp)
	add	1(sp)
	stax	7(sp)
	ldax	8(sp)
	adc	#0x00
	add	2(sp)
	stax	8(sp)
;	sdcc_hello.c: 77: if (c == '\r')
	ldax	3(sp)
	cpi	#0x0d
	bnz	00106$
;	sdcc_hello.c: 78: putchar('\n');
	ldi	#0x00
	push	a
	ldi	#0x0a
	push	a
	ads	#-2
	jal	_putchar
	ads	#4
00106$:
;	sdcc_hello.c: 79: if (c == '?')
	ldax	3(sp)
	cpi	#0x3f
	bnz	00108$
;	sdcc_hello.c: 80: print_lisa();
	jal	_print_lisa
00108$:
;	sdcc_hello.c: 81: if (c == 's')
	ldax	3(sp)
	cpi	#0x73
	bnz	00114$
;	sdcc_hello.c: 82: printf("\nCount: %u sum: %u\n", count, sum);
	ldax	8(sp)
	push	a
	ldax	8(sp)
	push	a
	ldax	7(sp)
	push	a
	ldax	7(sp)
	push	a
	ldi	#>(___str_3 + 0)
	push	a
	ldi	#<(___str_3 + 0)
	push	a
	ads	#-2
	jal	_printf
	ads	#8
	br	00114$
00116$:
;	sdcc_hello.c: 85: }
	ads	#8
	lra
	ret
	.area CODE (CODE)
	.area CONST (CODE,CDATA)
	.area CONST (CODE,CDATA)
_owl:
	.dw __str_4
	.dw __str_5
	.dw __str_6
	.dw __str_7
	.dw __str_8
	.dw __str_9
	.dw __str_10
	.dw __str_11
	.dw __str_12
	.dw __str_13
	.dw __str_14
	.dw #0x0000
	.area CODE (CODE)
	.area CONST (CODE,CDATA)
___str_0:
	.ascii "%s"
	.db 0x0a
	.db 0x00
	.area CODE (CODE)
	.area CONST (CODE,CDATA)
___str_1:
	.db 0x0a
	.ascii "Hello from TT07 LISA, compiled by sdcc -mlisa"
	.db 0x0a
	.db 0x00
	.area CODE (CODE)
	.area CONST (CODE,CDATA)
___str_2:
	.db 0x0a
	.ascii "sdcc_hello ready: ? = banner, s = count and sum"
	.db 0x0a
	.db 0x00
	.area CODE (CODE)
	.area CONST (CODE,CDATA)
___str_3:
	.db 0x0a
	.ascii "Count: %u sum: %u"
	.db 0x0a
	.db 0x00
	.area CODE (CODE)
	.area CONST (CODE,CDATA)
__str_4:
	.ascii "     .{{{}}}}}}."
	.db 0x00
	.area CODE (CODE)
	.area CONST (CODE,CDATA)
__str_5:
	.ascii "    {{{{{}}}}}}}."
	.db 0x00
	.area CODE (CODE)
	.area CONST (CODE,CDATA)
__str_6:
	.ascii "   {{{{  {{{{{}}}}"
	.db 0x00
	.area CODE (CODE)
	.area CONST (CODE,CDATA)
__str_7:
	.ascii "  }}}}} _   _ {{{{{"
	.db 0x00
	.area CODE (CODE)
	.area CONST (CODE,CDATA)
__str_8:
	.ascii "  }}}}  6   6  }}}}"
	.db 0x00
	.area CODE (CODE)
	.area CONST (CODE,CDATA)
__str_9:
	.ascii " {{{{C    ^    {{{{"
	.db 0x00
	.area CODE (CODE)
	.area CONST (CODE,CDATA)
__str_10:
	.ascii "}}}}}}"
	.db 0x5c
	.ascii "  '='  /}}}}}"
	.db 0x00
	.area CODE (CODE)
	.area CONST (CODE,CDATA)
__str_11:
	.ascii "{{{{{{{;.___.;}}}}}}"
	.db 0x00
	.area CODE (CODE)
	.area CONST (CODE,CDATA)
__str_12:
	.ascii " {{{{{{{)   (}}}}}}'"
	.db 0x00
	.area CODE (CODE)
	.area CONST (CODE,CDATA)
__str_13:
	.ascii "  ''"
	.db 0x22
	.ascii "''':   :''''''"
	.db 0x00
	.area CODE (CODE)
	.area CONST (CODE,CDATA)
__str_14:
	.ascii "  jgs    `@` "
	.db 0x00
	.area CODE (CODE)
	.area INITIALIZER (CODE,CDATA)
	.area CABS (ABS,CODE,CDATA)
