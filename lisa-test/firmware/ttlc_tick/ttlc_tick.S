/*
================================================================================
ttlc_tick.S:  LISA program that gives the TTLC a clock.

A free-running counter is written to PORT E, which the TTLC reads at its
addresses 128-135.  Timer 1 rolls over every 125 ms, so:

    TTLC address 128 (bit 0) toggles every 125 ms   -> a tick every 250 ms
                 129 (bit 1)                         -> every 500 ms
                 130 (bit 2)                         -> every 1 s
                 131 (bit 3)                         -> every 2 s

elevator6x2.asm reads 129.  The same count goes to PORT B (uo_out, when the
output mux leaves it to LISA) as a LISA-only sanity check.

Build:  make        (lisa_as + lisa_ld from the lisa-tools repository)

asmsyntax=lisa
================================================================================
*/

// LISA peripheral registers (the 0x200 bit selects the peripheral space)
#define  PORTB              0x201
#define  PORTE              0x205       // output to the TTLC
#define  TIMER1_PRE_DIVL    0x208
#define  TIMER1_PRE_DIVH    0x209
#define  TIMER1_DIVL        0x20A
#define  TIMER1_DIVH        0x20B
#define  TIMER1_CTRL        0x20C       // bit 0: enable; bit 7: rollover (reading clears it)
#define  TIMER1_TICK        0x20D

    .section .bss
    .public  gCount
gCount:
    .ds      1

    .section .text
    .public  main

main:
    // Timer 1: the pre-divider counts 50 MHz clocks, 49999 -> one count per
    // ms; the divisor counts those, 125 -> a rollover every 125 ms
    ldi     0
    sta     TIMER1_CTRL
    sta     TIMER1_DIVH
    sta     PORTE
    sta     PORTB
    sta     gCount
    ldi     0x4F
    sta     TIMER1_PRE_DIVL
    ldi     0xC3
    sta     TIMER1_PRE_DIVH
    ldi     125
    sta     TIMER1_DIVL
    ldi     1
    sta     TIMER1_CTRL             // enable
    ldi     0
    sta     TIMER1_TICK             // (re)start the count

_loop:
    lda     TIMER1_CTRL             // reading it clears the rollover flag
    andi    0x80
    bz      _loop

    ldc     0
    lda     gCount
    adc     1
    sta     gCount
    sta     PORTE
    sta     PORTB
    br      _loop

// vim: sw=4 ts=4 et
