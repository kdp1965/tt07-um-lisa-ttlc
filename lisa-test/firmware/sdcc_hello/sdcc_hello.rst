                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ISO C Compiler
                                      3 ; Version 4.5.2 #0 (Mac OS X ppc)
                                      4 ;--------------------------------------------------------
                                      5 	.module sdcc_hello
                                      6 	
                                      7 	.optsdcc -mlisa
                                      8 
                                      9 ; default segment ordering in RAM for linker
                                     10 	.area DATA
                                     11 	.area OSEG (OVR,DATA)
                                     12 
                                     13 ;--------------------------------------------------------
                                     14 ; Public variables in this module
                                     15 ;--------------------------------------------------------
                                     16 	.globl _main
                                     17 	.globl _printf
                                     18 	.globl _TIMER1_CTRL
                                     19 	.globl _PORTB
                                     20 	.globl _UART_STATUS
                                     21 	.globl _UART_RXTX
                                     22 	.globl _putchar
                                     23 ;--------------------------------------------------------
                                     24 ; special function registers
                                     25 ;--------------------------------------------------------
                                     26 	.area RSEG (ABS)
      000000                         27 	.org 0x0000
                           000210    28 _UART_RXTX	=	0x0210
                           000211    29 _UART_STATUS	=	0x0211
                           000201    30 _PORTB	=	0x0201
                           00020C    31 _TIMER1_CTRL	=	0x020c
                                     32 ;--------------------------------------------------------
                                     33 ; ram data
                                     34 ;--------------------------------------------------------
                                     35 	.area DATA
                                     36 ;--------------------------------------------------------
                                     37 ; ram data
                                     38 ;--------------------------------------------------------
                                     39 	.area INITIALIZED
                                     40 ;--------------------------------------------------------
                                     41 ; overlayable items in ram
                                     42 ;--------------------------------------------------------
                                     43 ;--------------------------------------------------------
                                     44 ; Stack segment in internal ram
                                     45 ;--------------------------------------------------------
                                     46 	.area SSEG
      000000                         47 __start__stack:
      000000                         48 	.ds	1
                                     49 
                                     50 ;--------------------------------------------------------
                                     51 ; absolute external ram data
                                     52 ;--------------------------------------------------------
                                     53 	.area DABS (ABS)
                                     54 ;--------------------------------------------------------
                                     55 ; interrupt vector
                                     56 ;--------------------------------------------------------
                                     57 	.area HOME (CODE)
      000000                         58 __interrupt_vect:
      000000 0C 00                   59 	jal	__sdcc_gsinit_startup
      000002 40 8B                   60 	rets
      000004 40 8B                   61 	rets
      000006 40 8B                   62 	rets
      000008 40 8B                   63 	rets
      00000A 40 8B                   64 	rets
      00000C 40 8B                   65 	rets
      00000E 40 8B                   66 	rets
      000010 40 8B                   67 	rets
      000012 40 8B                   68 	rets
                                     69 ;--------------------------------------------------------
                                     70 ; global & static initialisations
                                     71 ;--------------------------------------------------------
                                     72 	.area HOME (CODE)
                                     73 	.area GSINIT (CODE)
                                     74 	.area GSFINAL (CODE)
                                     75 	.area GSINIT (CODE)
                                     76 	.area GSINIT (CODE)
      000018                         77 __sdcc_gsinit_startup::
      000018 80 A1 7F 00             78 	ldx	#0x007f
      00001C C8 8A                   79 	xchg	sp
      00001E 41 A1                   80 	amode	1
      000020 EE 00                   81 	jal	___sdcc_external_startup
      000022 00 A4                   82 	cpi	#0
      000024 01 A2                   83 	if	ne
      000026 0A 00                   84 	jal	__sdcc_program_startup
      000028 00 80                   85 	ldi	#>l_DATA
      00002A 80 A0                   86 	push	a
      00002C 00 80                   87 	ldi	#<l_DATA
      00002E 80 A0                   88 	push	a
      000030 80 A1 00 00             89 	ldx	#s_DATA
      000034                         90 00001$:
      000034 01 F2                   91 	ldax	1(sp)
      000036 02 DA                   92 	or	2(sp)
      000038 08 B8                   93 	bz	00002$
      00003A 00 80                   94 	ldi	#0
      00003C 00 F8                   95 	stax	0(ix)
      00003E 01 98                   96 	adx	#1
      000040 01 9E                   97 	dcx	1(sp)
      000042 03 A2                   98 	if	c
      000044 02 9E                   99 	dcx	2(sp)
      000046 F7 B7                  100 	br	00001$
      000048                        101 00002$:
      000048 00 80                  102 	ldi	#>s_INITIALIZED
      00004A 80 A0                  103 	push	a
      00004C 00 80                  104 	ldi	#<s_INITIALIZED
      00004E 80 A0                  105 	push	a
      000050 00 80                  106 	ldi	#>l_INITIALIZED
      000052 04 FA                  107 	stax	4(sp)
      000054 00 80                  108 	ldi	#<l_INITIALIZED
      000056 03 FA                  109 	stax	3(sp)
      000058 80 A1 46 8C            110 	ldx	#s_INITIALIZER
      00005C                        111 00003$:
      00005C 03 F2                  112 	ldax	3(sp)
      00005E 04 DA                  113 	or	4(sp)
      000060 0E B8                  114 	bz	00004$
      000062 80 8A                  115 	call	ix
      000064 01 98                  116 	adx	#1
      000066 68 A1                  117 	push	ix
      000068 03 CC                  118 	ldxx	3(sp)
      00006A 00 F8                  119 	stax	0(ix)
      00006C 03 E6                  120 	inx	3(sp)
      00006E 03 A2                  121 	if	c
      000070 04 E6                  122 	inx	4(sp)
      000072 6C A1                  123 	pop	ix
      000074 03 9E                  124 	dcx	3(sp)
      000076 03 A2                  125 	if	c
      000078 04 9E                  126 	dcx	4(sp)
      00007A F1 B7                  127 	br	00003$
      00007C                        128 00004$:
      00007C 04 94                  129 	ads	#4
                                    130 	.area GSFINAL (CODE)
      00007E 0A 00                  131 	jal	__sdcc_program_startup
                                    132 ;--------------------------------------------------------
                                    133 ; Home
                                    134 ;--------------------------------------------------------
                                    135 	.area HOME (CODE)
                                    136 	.area HOME (CODE)
      000014                        137 __sdcc_program_startup:
      000014 8D 00                  138 	jal	_main
      000016                        139 00001$:
      000016 00 B0                  140 	br	00001$
                                    141 ;	return from main will return to caller
                                    142 ;--------------------------------------------------------
                                    143 ; code
                                    144 ;--------------------------------------------------------
                                    145 	.area CODE (CODE)
                                    146 ;	sdcc_hello.c: 20: int putchar(int c)
                                    147 ;	---------------------------------
                                    148 ;	 Function putchar
                                    149 ;	---------------------------------
      000080                        150 _putchar:
      000080 FF 97                  151 	ads	#-1
                                    152 ;	sdcc_hello.c: 22: while (!(UART_STATUS & UART_TX_EMPTY))
      000082                        153 00101$:
      000082 11 F6                  154 	lda	_UART_STATUS
      000084 01 FA                  155 	stax	1(sp)
      000086 02 D4                  156 	andi	#0x02
      000088 FD BF                  157 	bz	00101$
                                    158 ;	sdcc_hello.c: 24: UART_RXTX = c;
      00008A 04 F2                  159 	ldax	4(sp)
      00008C 10 FE                  160 	sta	_UART_RXTX
                                    161 ;	sdcc_hello.c: 25: return c;
      00008E 04 F2                  162 	ldax	4(sp)
      000090 02 FA                  163 	stax	2(sp)
      000092 05 F2                  164 	ldax	5(sp)
      000094 03 FA                  165 	stax	3(sp)
      000096                        166 00104$:
                                    167 ;	sdcc_hello.c: 26: }
      000096 01 94                  168 	ads	#1
      000098 00 8A                  169 	ret
                                    170 ;	sdcc_hello.c: 43: static void print_lisa(void)
                                    171 ;	---------------------------------
                                    172 ;	 Function print_lisa
                                    173 ;	---------------------------------
      00009A                        174 _print_lisa:
      00009A 60 A1                  175 	sra
      00009C FC 97                  176 	ads	#-4
                                    177 ;	sdcc_hello.c: 46: putchar('\n');
      00009E 00 80                  178 	ldi	#0x00
      0000A0 80 A0                  179 	push	a
      0000A2 0A 80                  180 	ldi	#0x0a
      0000A4 80 A0                  181 	push	a
      0000A6 FE 97                  182 	ads	#-2
      0000A8 40 00                  183 	jal	_putchar
      0000AA 04 94                  184 	ads	#4
                                    185 ;	sdcc_hello.c: 47: for (p = owl; *p; p++)
      0000AC B3 80                  186 	ldi	#<(_owl + 0)
      0000AE 03 FA                  187 	stax	3(sp)
      0000B0 84 80                  188 	ldi	#>(_owl + 0)
      0000B2 04 FA                  189 	stax	4(sp)
      0000B4                        190 00103$:
      0000B4 03 CC                  191 	ldxx	3(sp)
      0000B6 18 A0                  192 	txau
      0000B8 47 A0                  193 	btst	7
      0000BA 06 B8                  194 	bz	00120$
      0000BC 00 F0                  195 	ldax	0(ix)
      0000BE 01 FA                  196 	stax	1(sp)
      0000C0 01 F0                  197 	ldax	1(ix)
      0000C2 02 FA                  198 	stax	2(sp)
      0000C4 0B B0                  199 	br	00121$
      0000C6                        200 00120$:
      0000C6 18 A0                  201 	txau
      0000C8 7F D4                  202 	andi	#0x7f
      0000CA A1 A1                  203 	addaxu
      0000CC 10 A0                  204 	txa
      0000CE A0 A1                  205 	addax
      0000D0 80 8A                  206 	call	ix
      0000D2 01 98                  207 	adx	#1
      0000D4 01 FA                  208 	stax	1(sp)
      0000D6 80 8A                  209 	call	ix
      0000D8 02 FA                  210 	stax	2(sp)
      0000DA                        211 00121$:
      0000DA 01 F2                  212 	ldax	1(sp)
      0000DC 02 DA                  213 	or	2(sp)
      0000DE 14 B8                  214 	bz	00101$
                                    215 ;	sdcc_hello.c: 48: printf("%s\n", *p);
      0000E0 02 F2                  216 	ldax	2(sp)
      0000E2 80 A0                  217 	push	a
      0000E4 02 F2                  218 	ldax	2(sp)
      0000E6 80 A0                  219 	push	a
      0000E8 84 80                  220 	ldi	#>(___str_0 + 0)
      0000EA 80 A0                  221 	push	a
      0000EC CB 80                  222 	ldi	#<(___str_0 + 0)
      0000EE 80 A0                  223 	push	a
      0000F0 FE 97                  224 	ads	#-2
      0000F2 1F 01                  225 	jal	_printf
      0000F4 06 94                  226 	ads	#6
                                    227 ;	sdcc_hello.c: 47: for (p = owl; *p; p++)
      0000F6 03 F2                  228 	ldax	3(sp)
      0000F8 08 A0                  229 	ldc	#0
      0000FA 02 90                  230 	adc	#0x02
      0000FC 03 FA                  231 	stax	3(sp)
      0000FE 04 F2                  232 	ldax	4(sp)
      000100 00 90                  233 	adc	#0x00
      000102 04 FA                  234 	stax	4(sp)
      000104 D8 B7                  235 	br	00103$
      000106                        236 00101$:
                                    237 ;	sdcc_hello.c: 49: printf("\nHello from TT07 LISA, compiled by sdcc -mlisa\n");
      000106 84 80                  238 	ldi	#>(___str_1 + 0)
      000108 80 A0                  239 	push	a
      00010A CF 80                  240 	ldi	#<(___str_1 + 0)
      00010C 80 A0                  241 	push	a
      00010E FE 97                  242 	ads	#-2
      000110 1F 01                  243 	jal	_printf
      000112 04 94                  244 	ads	#4
      000114                        245 00105$:
                                    246 ;	sdcc_hello.c: 50: }
      000114 04 94                  247 	ads	#4
      000116 64 A1                  248 	lra
      000118 00 8A                  249 	ret
                                    250 ;	sdcc_hello.c: 52: void main(void)
                                    251 ;	---------------------------------
                                    252 ;	 Function main
                                    253 ;	---------------------------------
      00011A                        254 _main:
      00011A 60 A1                  255 	sra
      00011C F8 97                  256 	ads	#-8
                                    257 ;	sdcc_hello.c: 55: unsigned int sum = 0;
      00011E 00 80                  258 	ldi	#0x00
      000120 07 FA                  259 	stax	7(sp)
      000122 08 FA                  260 	stax	8(sp)
                                    261 ;	sdcc_hello.c: 56: unsigned char ticks = 0;
      000124 00 80                  262 	ldi	#0x00
      000126 06 FA                  263 	stax	6(sp)
                                    264 ;	sdcc_hello.c: 59: PORTB = 0x4f;
      000128 4F 80                  265 	ldi	#0x4f
      00012A 01 FE                  266 	sta	_PORTB
                                    267 ;	sdcc_hello.c: 60: printf("\nsdcc_hello ready: ? = banner, s = count and sum\n");
      00012C 84 80                  268 	ldi	#>(___str_2 + 0)
      00012E 80 A0                  269 	push	a
      000130 FF 80                  270 	ldi	#<(___str_2 + 0)
      000132 80 A0                  271 	push	a
      000134 FE 97                  272 	ads	#-2
      000136 1F 01                  273 	jal	_printf
      000138 04 94                  274 	ads	#4
      00013A 00 80                  275 	ldi	#0x00
      00013C 04 FA                  276 	stax	4(sp)
      00013E 05 FA                  277 	stax	5(sp)
      000140                        278 00114$:
                                    279 ;	sdcc_hello.c: 63: count++;
      000140 04 E6                  280 	inx	4(sp)
      000142 03 A2                  281 	if	c
      000144 05 E6                  282 	inx.p	5(sp)
                                    283 ;	sdcc_hello.c: 64: if (TIMER1_CTRL & TIMER_ROLLOVER)
      000146 0C F6                  284 	lda	_TIMER1_CTRL
      000148 03 FA                  285 	stax	3(sp)
      00014A 80 D4                  286 	andi	#0x80
      00014C 0E B8                  287 	bz	00104$
                                    288 ;	sdcc_hello.c: 66: if (++ticks == 4)
      00014E 06 E6                  289 	inx	6(sp)
      000150 06 F2                  290 	ldax	6(sp)
      000152 04 A4                  291 	cpi	#0x04
      000154 0A A8                  292 	bnz	00104$
                                    293 ;	sdcc_hello.c: 68: ticks = 0;
      000156 00 80                  294 	ldi	#0x00
      000158 06 FA                  295 	stax	6(sp)
                                    296 ;	sdcc_hello.c: 69: PORTB ^= 0x08;
      00015A 01 F6                  297 	lda	_PORTB
      00015C 80 A0                  298 	push	a
      00015E 08 80                  299 	ldi	#0x08
      000160 01 EE                  300 	swap	1(sp)
      000162 01 E2                  301 	xor	1(sp)
      000164 01 94                  302 	ads	#1
      000166 01 FE                  303 	sta	_PORTB
      000168                        304 00104$:
                                    305 ;	sdcc_hello.c: 72: if (UART_STATUS & UART_RX_AVAIL)
      000168 11 F6                  306 	lda	_UART_STATUS
      00016A 03 FA                  307 	stax	3(sp)
      00016C 01 D4                  308 	andi	#0x01
      00016E E9 BF                  309 	bz	00114$
                                    310 ;	sdcc_hello.c: 74: c = UART_RXTX;
      000170 10 F6                  311 	lda	_UART_RXTX
                                    312 ;	sdcc_hello.c: 75: putchar(c);
      000172 03 FA                  313 	stax	3(sp)
      000174 01 FA                  314 	stax	1(sp)
      000176 00 80                  315 	ldi	#0x00
      000178 02 FA                  316 	stax	2(sp)
      00017A 80 A0                  317 	push	a
      00017C 02 F2                  318 	ldax	2(sp)
      00017E 80 A0                  319 	push	a
      000180 FE 97                  320 	ads	#-2
      000182 40 00                  321 	jal	_putchar
      000184 04 94                  322 	ads	#4
                                    323 ;	sdcc_hello.c: 76: sum += c;
      000186 07 F2                  324 	ldax	7(sp)
      000188 01 C2                  325 	add	1(sp)
      00018A 07 FA                  326 	stax	7(sp)
      00018C 08 F2                  327 	ldax	8(sp)
      00018E 00 90                  328 	adc	#0x00
      000190 02 C2                  329 	add	2(sp)
      000192 08 FA                  330 	stax	8(sp)
                                    331 ;	sdcc_hello.c: 77: if (c == '\r')
      000194 03 F2                  332 	ldax	3(sp)
      000196 0D A4                  333 	cpi	#0x0d
      000198 08 A8                  334 	bnz	00106$
                                    335 ;	sdcc_hello.c: 78: putchar('\n');
      00019A 00 80                  336 	ldi	#0x00
      00019C 80 A0                  337 	push	a
      00019E 0A 80                  338 	ldi	#0x0a
      0001A0 80 A0                  339 	push	a
      0001A2 FE 97                  340 	ads	#-2
      0001A4 40 00                  341 	jal	_putchar
      0001A6 04 94                  342 	ads	#4
      0001A8                        343 00106$:
                                    344 ;	sdcc_hello.c: 79: if (c == '?')
      0001A8 03 F2                  345 	ldax	3(sp)
      0001AA 3F A4                  346 	cpi	#0x3f
      0001AC 02 A8                  347 	bnz	00108$
                                    348 ;	sdcc_hello.c: 80: print_lisa();
      0001AE 4D 00                  349 	jal	_print_lisa
      0001B0                        350 00108$:
                                    351 ;	sdcc_hello.c: 81: if (c == 's')
      0001B0 03 F2                  352 	ldax	3(sp)
      0001B2 73 A4                  353 	cpi	#0x73
      0001B4 C6 AF                  354 	bnz	00114$
                                    355 ;	sdcc_hello.c: 82: printf("\nCount: %u sum: %u\n", count, sum);
      0001B6 08 F2                  356 	ldax	8(sp)
      0001B8 80 A0                  357 	push	a
      0001BA 08 F2                  358 	ldax	8(sp)
      0001BC 80 A0                  359 	push	a
      0001BE 07 F2                  360 	ldax	7(sp)
      0001C0 80 A0                  361 	push	a
      0001C2 07 F2                  362 	ldax	7(sp)
      0001C4 80 A0                  363 	push	a
      0001C6 85 80                  364 	ldi	#>(___str_3 + 0)
      0001C8 80 A0                  365 	push	a
      0001CA 31 80                  366 	ldi	#<(___str_3 + 0)
      0001CC 80 A0                  367 	push	a
      0001CE FE 97                  368 	ads	#-2
      0001D0 1F 01                  369 	jal	_printf
      0001D2 08 94                  370 	ads	#8
      0001D4 B6 B7                  371 	br	00114$
      0001D6                        372 00116$:
                                    373 ;	sdcc_hello.c: 85: }
      0001D6 08 94                  374 	ads	#8
      0001D8 64 A1                  375 	lra
      0001DA 00 8A                  376 	ret
                                    377 	.area CODE (CODE)
                                    378 	.area CONST (CODE,CDATA)
                                    379 	.area CONST (CODE,CDATA)
      0012CC                        380 _owl:
      0012CC 45 80 00 8A 85 80 00   381 	.dw __str_4
             8A
      0012D4 56 80 00 8A 85 80 00   382 	.dw __str_5
             8A
      0012DC 68 80 00 8A 85 80 00   383 	.dw __str_6
             8A
      0012E4 7B 80 00 8A 85 80 00   384 	.dw __str_7
             8A
      0012EC 8F 80 00 8A 85 80 00   385 	.dw __str_8
             8A
      0012F4 A3 80 00 8A 85 80 00   386 	.dw __str_9
             8A
      0012FC B7 80 00 8A 85 80 00   387 	.dw __str_10
             8A
      001304 CC 80 00 8A 85 80 00   388 	.dw __str_11
             8A
      00130C E1 80 00 8A 85 80 00   389 	.dw __str_12
             8A
      001314 F6 80 00 8A 85 80 00   390 	.dw __str_13
             8A
      00131C 0A 80 00 8A 86 80 00   391 	.dw __str_14
             8A
      001324 00 80 00 8A 00 80 00   392 	.dw #0x0000
             8A
                                    393 	.area CODE (CODE)
                                    394 	.area CONST (CODE,CDATA)
      00132C                        395 ___str_0:
      00132C 25 80 00 8A 73 80 00   396 	.ascii "%s"
             8A
      001334 0A 80 00 8A            397 	.db 0x0a
      001338 00 80 00 8A            398 	.db 0x00
                                    399 	.area CODE (CODE)
                                    400 	.area CONST (CODE,CDATA)
      00133C                        401 ___str_1:
      00133C 0A 80 00 8A            402 	.db 0x0a
      001340 48 80 00 8A 65 80 00   403 	.ascii "Hello from TT07 LISA, compiled by sdcc -mlisa"
             8A 6C 80 00 8A 6C 80
             00 8A 6F 80 00 8A 20
             80 00 8A 66 80 00 8A
             72 80 00 8A 6F 80 00
             8A 6D 80 00 8A 20 80
             00 8A 54 80 00 8A 54
             80 00 8A 30 80 00 8A
             37 80 00 8A 20 80 00
             8A 4C 80 00 8A 49 80
             00 8A 53 80 00 8A 41
             80 00 8A 2C 80 00 8A
             20 80 00 8A 63 80 00
             8A 6F 80 00 8A 6D 80
             00 8A 70 80 00 8A 69
             80 00 8A 6C 80 00 8A
             65 80 00 8A 64 80 00
             8A 20 80 00 8A 62 80
             00 8A
      0013C0 79 80 00 8A            404 	.db 0x0a
      0013C4 20 80 00 8A            405 	.db 0x00
                                    406 	.area CODE (CODE)
                                    407 	.area CONST (CODE,CDATA)
      000130                        408 ___str_2:
      0013C8 73 80 00 8A            409 	.db 0x0a
      0013CC 64 80 00 8A 63 80 00   410 	.ascii "sdcc_hello ready: ? = banner, s = count and sum"
             8A 63 80 00 8A 20 80
             00 8A 2D 80 00 8A 6D
             80 00 8A 6C 80 00 8A
             69 80 00 8A 73 80 00
             8A 61 80 00 8A 0A 80
             00 8A 00 80 00 8A 65
             80 00 8A 61 80 00 8A
             64 80 00 8A 79 80 00
             8A 3A 80 00 8A 20 80
             00 8A 3F 80 00 8A 20
             80 00 8A 3D 80 00 8A
             20 80 00 8A 62 80 00
             8A 61 80 00 8A 6E 80
             00 8A 6E 80 00 8A 65
             80 00 8A 72 80 00 8A
             2C 80 00 8A 20 80 00
             8A 73 80 00 8A 20 80
             00 8A
      0013FC 0A 80 00 8A            411 	.db 0x0a
      0013FC 0A 80 00 8A            412 	.db 0x00
                                    413 	.area CODE (CODE)
                                    414 	.area CONST (CODE,CDATA)
      0001F8                        415 ___str_3:
      001400 73 80 00 8A            416 	.db 0x0a
      001404 64 80 00 8A 63 80 00   417 	.ascii "Count: %u sum: %u"
             8A 63 80 00 8A 5F 80
             00 8A 68 80 00 8A 65
             80 00 8A 6C 80 00 8A
             6C 80 00 8A 6F 80 00
             8A 20 80 00 8A 72 80
             00 8A 65 80 00 8A 61
             80 00 8A 64 80 00 8A
             79 80 00 8A 3A 80 00
             8A 20 80 00 8A
      001448 3F 80 00 8A            418 	.db 0x0a
      00144C 20 80 00 8A            419 	.db 0x00
                                    420 	.area CODE (CODE)
                                    421 	.area CONST (CODE,CDATA)
      000248                        422 __str_4:
      001450 3D 80 00 8A 20 80 00   423 	.ascii "     .{{{}}}}}}."
             8A 62 80 00 8A 61 80
             00 8A 6E 80 00 8A 6E
             80 00 8A 65 80 00 8A
             72 80 00 8A 2C 80 00
             8A 20 80 00 8A 73 80
             00 8A 20 80 00 8A 3D
             80 00 8A 20 80 00 8A
             63 80 00 8A 6F 80 00
             8A
      001490 75 80 00 8A            424 	.db 0x00
                                    425 	.area CODE (CODE)
                                    426 	.area CONST (CODE,CDATA)
      00028C                        427 __str_5:
      001494 6E 80 00 8A 74 80 00   428 	.ascii "    {{{{{}}}}}}}."
             8A 20 80 00 8A 61 80
             00 8A 6E 80 00 8A 64
             80 00 8A 20 80 00 8A
             73 80 00 8A 75 80 00
             8A 6D 80 00 8A 0A 80
             00 8A 00 80 00 8A 7D
             80 00 8A 7D 80 00 8A
             7D 80 00 8A 7D 80 00
             8A 2E 80 00 8A
      0014C4 00 80 00 8A            429 	.db 0x00
                                    430 	.area CODE (CODE)
                                    431 	.area CONST (CODE,CDATA)
      0002D4                        432 __str_6:
      0014C4 0A 80 00 8A 43 80 00   433 	.ascii "   {{{{  {{{{{}}}}"
             8A 6F 80 00 8A 75 80
             00 8A 6E 80 00 8A 74
             80 00 8A 3A 80 00 8A
             20 80 00 8A 25 80 00
             8A 75 80 00 8A 20 80
             00 8A 73 80 00 8A 75
             80 00 8A 6D 80 00 8A
             3A 80 00 8A 20 80 00
             8A 25 80 00 8A 75 80
             00 8A
      00150C 0A 80 00 8A            434 	.db 0x00
                                    435 	.area CODE (CODE)
                                    436 	.area CONST (CODE,CDATA)
      000320                        437 __str_7:
      001510 00 80 00 8A 20 80 00   438 	.ascii "  }}}}} _   _ {{{{{"
             8A 7D 80 00 8A 7D 80
             00 8A 7D 80 00 8A 7D
             80 00 8A 7D 80 00 8A
             20 80 00 8A 5F 80 00
             8A 20 80 00 8A 20 80
             00 8A 20 80 00 8A 5F
             80 00 8A 20 80 00 8A
             7B 80 00 8A 7B 80 00
             8A 7B 80 00 8A 7B 80
             00 8A 7B 80 00 8A
      001514 00 80 00 8A            439 	.db 0x00
                                    440 	.area CODE (CODE)
                                    441 	.area CONST (CODE,CDATA)
      000370                        442 __str_8:
      001514 20 80 00 8A 20 80 00   443 	.ascii "  }}}}  6   6  }}}}"
             8A 20 80 00 8A 20 80
             00 8A 20 80 00 8A 2E
             80 00 8A 7B 80 00 8A
             7B 80 00 8A 7B 80 00
             8A 7D 80 00 8A 7D 80
             00 8A 7D 80 00 8A 7D
             80 00 8A 7D 80 00 8A
             7D 80 00 8A 2E 80 00
             8A 00 80 00 8A 7D 80
             00 8A 7D 80 00 8A
      001558 00 80 00 8A            444 	.db 0x00
                                    445 	.area CODE (CODE)
                                    446 	.area CONST (CODE,CDATA)
      0003C0                        447 __str_9:
      001558 20 80 00 8A 20 80 00   448 	.ascii " {{{{C    ^    {{{{"
             8A 20 80 00 8A 20 80
             00 8A 7B 80 00 8A 7B
             80 00 8A 7B 80 00 8A
             7B 80 00 8A 7B 80 00
             8A 7D 80 00 8A 7D 80
             00 8A 7D 80 00 8A 7D
             80 00 8A 7D 80 00 8A
             7D 80 00 8A 7D 80 00
             8A 2E 80 00 8A 00 80
             00 8A 7B 80 00 8A
      0015A0 00 80 00 8A            449 	.db 0x00
                                    450 	.area CODE (CODE)
                                    451 	.area CONST (CODE,CDATA)
      000410                        452 __str_10:
      0015A0 20 80 00 8A 20 80 00   453 	.ascii "}}}}}}"
             8A 20 80 00 8A 7B 80
             00 8A 7B 80 00 8A 7B
             80 00 8A
      0015B8 7B 80 00 8A            454 	.db 0x5c
      0015BC 20 80 00 8A 20 80 00   455 	.ascii "  '='  /}}}}}"
             8A 7B 80 00 8A 7B 80
             00 8A 7B 80 00 8A 7B
             80 00 8A 7B 80 00 8A
             7D 80 00 8A 7D 80 00
             8A 7D 80 00 8A 7D 80
             00 8A 00 80 00 8A 7D
             80 00 8A
      0015EC 00 80 00 8A            456 	.db 0x00
                                    457 	.area CODE (CODE)
                                    458 	.area CONST (CODE,CDATA)
      000464                        459 __str_11:
      0015EC 20 80 00 8A 20 80 00   460 	.ascii "{{{{{{{;.___.;}}}}}}"
             8A 7D 80 00 8A 7D 80
             00 8A 7D 80 00 8A 7D
             80 00 8A 7D 80 00 8A
             20 80 00 8A 5F 80 00
             8A 20 80 00 8A 20 80
             00 8A 20 80 00 8A 5F
             80 00 8A 20 80 00 8A
             7B 80 00 8A 7B 80 00
             8A 7B 80 00 8A 7B 80
             00 8A 7B 80 00 8A 00
             80 00 8A
      00163C 00 80 00 8A            461 	.db 0x00
                                    462 	.area CODE (CODE)
                                    463 	.area CONST (CODE,CDATA)
      0004B8                        464 __str_12:
      00163C 20 80 00 8A 20 80 00   465 	.ascii " {{{{{{{)   (}}}}}}'"
             8A 7D 80 00 8A 7D 80
             00 8A 7D 80 00 8A 7D
             80 00 8A 20 80 00 8A
             20 80 00 8A 36 80 00
             8A 20 80 00 8A 20 80
             00 8A 20 80 00 8A 36
             80 00 8A 20 80 00 8A
             20 80 00 8A 7D 80 00
             8A 7D 80 00 8A 7D 80
             00 8A 7D 80 00 8A 00
             80 00 8A
      00168C 00 80 00 8A            466 	.db 0x00
                                    467 	.area CODE (CODE)
                                    468 	.area CONST (CODE,CDATA)
      00050C                        469 __str_13:
      00168C 20 80 00 8A 7B 80 00   470 	.ascii "  ''"
             8A 7B 80 00 8A 7B 80
             00 8A
      00169C 7B 80 00 8A            471 	.db 0x22
      0016A0 43 80 00 8A 20 80 00   472 	.ascii "''':   :''''''"
             8A 20 80 00 8A 20 80
             00 8A 20 80 00 8A 5E
             80 00 8A 20 80 00 8A
             20 80 00 8A 20 80 00
             8A 20 80 00 8A 7B 80
             00 8A 7B 80 00 8A 7B
             80 00 8A 7B 80 00 8A
      0016D8 00 80 00 8A            473 	.db 0x00
                                    474 	.area CODE (CODE)
                                    475 	.area CONST (CODE,CDATA)
      0016DC                        476 __str_14:
      0016DC 7D 80 00 8A 7D 80 00   477 	.ascii "  jgs    `@` "
             8A 7D 80 00 8A 7D 80
             00 8A 7D 80 00 8A 7D
             80 00 8A 5C 80 00 8A
             20 80 00 8A 20 80 00
             8A 27 80 00 8A 3D 80
             00 8A 27 80 00 8A 20
             80 00 8A
      001710 20 80 00 8A            478 	.db 0x00
                                    479 	.area CODE (CODE)
                                    480 	.area INITIALIZER (CODE,CDATA)
                                    481 	.area CABS (ABS,CODE,CDATA)
