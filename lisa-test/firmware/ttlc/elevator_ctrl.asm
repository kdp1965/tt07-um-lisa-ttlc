/*
================================================================================
This is an eleveator control program

asmsyntax=mc14500b
================================================================================
*/

// Include our I/O definitions
include <plc_config.h>

loop:
    // ==================================================
    // Perform I/O operation
    // ==================================================
    nopo

    // ==================================================
    // Manage the UP1 button
    // ==================================================
    ld      UP1                 // RR = UP1 button
    or      UP1_HOLD
    sto     UP1_LED             // Update the LED
    sto     UP1_HOLD            // Save in case newly pressed

    // ==================================================
    // Manage the UP2 button
    // ==================================================
    ld      UP2                 // RR = UP2 button
    orc     UP2_HOLD
    sto     UP2_LED             // Update the LED
    sto     UP2_HOLD            // Save in case newly pressed

    // ==================================================
    // Manage the UP3 button
    // ==================================================
    ld      UP3                 // RR = UP3 button
    or      UP3_HOLD
    sto     UP3_LED             // Update the LED
    sto     UP3_HOLD            // Save in case newly pressed

    jmp     test_jump

    xnor    RR
    sto     16  
    ld      TIMER1_ACTIVE       // Load storage if TIMER1 active
    skz
    jmp     check_timer1

    nopf

    nopo
    nopo
    nopo

check_timer1:
    rtn

test_jump:
    xnor    RR
    sto     127
    rtn

// sw=4 ts=4 et

