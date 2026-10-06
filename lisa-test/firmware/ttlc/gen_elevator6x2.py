#!/usr/bin/env python3
'''
Writes elevator6x2.asm: a 6-floor, 2-car collective elevator controller for
the TTLC (MC14500B).  The two cars run identical logic on different I/O
addresses, and the MC14500B has no indirect addressing, so the per-car code
is emitted twice from one template.  See elevator6x2.h for the I/O map.

    python3 gen_elevator6x2.py && python3 ../../../../mc14500b_asm/mc14500_as.py elevator6x2.asm
'''

FLOORS = 6
HALL = {   # floor -> hall call indicators (output addresses) at that floor
    0: ['HALL_UP0'], 1: ['HALL_UP1', 'HALL_DN1'], 2: ['HALL_UP2', 'HALL_DN2'],
    3: ['HALL_UP3', 'HALL_DN3'], 4: ['HALL_UP4', 'HALL_DN4'], 5: ['HALL_DN5'],
}
BUTTON = {   # floor -> hall buttons (input addresses) at that floor
    0: ['BTN_UP0'], 1: ['BTN_UP1', 'BTN_DN1'], 2: ['BTN_UP2', 'BTN_DN2'],
    3: ['BTN_UP3', 'BTN_DN3'], 4: ['BTN_UP4', 'BTN_DN4'], 5: ['BTN_DN5'],
}

out = []
def emit(line=''): out.append(line)
def op(mnemonic, arg=None, comment=''):
    text = f'    {mnemonic:<6} {arg if arg is not None else "":<16}'
    emit((text + ('// ' + comment if comment else '')).rstrip())
def section(title):
    emit(); emit('    // ' + '-' * 70); emit('    // ' + title); emit('    // ' + '-' * 70)


def car(n):
    '''The per-car logic.  n = 1 or 2.'''
    other = 3 - n
    POS, OPOS, CAB = f'POS{n}', f'POS{other}', f'CAB{n}'
    DIR, DWELL, DWELL2 = f'C{n}_DIR', f'C{n}_DWELL', f'C{n}_DWELL2'
    DOOR, LUP, LDN = f'DOOR{n}', f'UP{n}', f'DOWN{n}'

    section(f'car {n}: start at floor 0 when no position bit is set yet (after reset)')
    for f in range(FLOORS):
        op('ld' if f == 0 else 'or', f'{POS}+{f}')
    op('ldc', 'RR', 'RR = position unknown')
    op('or', f'{POS}+0')
    op('sto', f'{POS}+0')

    section(f'car {n}: requests that concern this car, per floor')
    emit(f'    // R_f = cabin button f | hall calls at f' +
         (' (unless car 1 is there: it takes them)' if n == 2 else ''))
    for f in range(FLOORS):
        first = True
        for h in HALL[f]:
            op('ld' if first else 'or', h); first = False
        if n == 2:
            op('andc', f'{OPOS}+{f}')
        op('or', f'{CAB}+{f}')
        op('sto', f'R{f}')

    section(f'car {n}: is there a request above / below the current floor?')
    for f in range(FLOORS - 2, -1, -1):              # ANY_ABOVE: floors above f, when at f
        first = True
        for g in range(f + 1, FLOORS):
            op('ld' if first else 'or', f'R{g}'); first = False
        op('and', f'{POS}+{f}')
        if f != FLOORS - 2:
            op('or', 'ANY_ABOVE')
        op('sto', 'ANY_ABOVE')
    for f in range(1, FLOORS):                        # ANY_BELOW: floors below f, when at f
        first = True
        for g in range(0, f):
            op('ld' if first else 'or', f'R{g}'); first = False
        op('and', f'{POS}+{f}')
        if f != 1:
            op('or', 'ANY_BELOW')
        op('sto', 'ANY_BELOW')

    section(f'car {n}: is there a request at the current floor?')
    for f in range(FLOORS):
        op('ld', f'R{f}')
        op('and', f'{POS}+{f}')
        if f:
            op('or', 'REQ_HERE')
        op('sto', 'REQ_HERE')

    section(f'car {n}: what this tick does (everything is gated by TICK_EDGE)')
    emit('    // STAY    = tick & DWELL2                      first dwell tick: door stays open')
    emit('    // CLOSING = tick & DWELL & !DWELL2             second dwell tick: door closes')
    emit('    // SERVICE = tick & !DWELL & REQ_HERE           a request here: open the door')
    emit('    // MV_UP   = tick & !DWELL & !SERVICE & ANY_ABOVE & (DIR | !ANY_BELOW)')
    emit('    // MV_DN   = tick & !DWELL & !SERVICE & ANY_BELOW & !MV_UP')
    op('ld', 'TICK_EDGE'); op('and', DWELL2); op('sto', 'STAY')
    op('ld', 'TICK_EDGE'); op('and', DWELL); op('andc', DWELL2); op('sto', 'CLOSING')
    op('ld', 'TICK_EDGE'); op('andc', DWELL); op('and', 'REQ_HERE'); op('sto', 'SERVICE')
    op('ld', DIR); op('orc', 'ANY_BELOW'); op('and', 'ANY_ABOVE'); op('and', 'TICK_EDGE')
    op('andc', DWELL); op('andc', 'SERVICE'); op('sto', 'MV_UP')
    op('ldc', 'MV_UP'); op('and', 'ANY_BELOW'); op('and', 'TICK_EDGE')
    op('andc', DWELL); op('andc', 'SERVICE'); op('sto', 'MV_DN')

    section(f'car {n}: door dwell (OEN gates the stores: only the chosen action writes)')
    op('oen', 'STAY', 'first dwell tick over')
    op('ld', 'ZERO'); op('sto', DWELL2)
    op('oen', 'CLOSING', 'second dwell tick: close the door')
    op('ld', 'ZERO'); op('sto', DWELL); op('sto', DOOR)
    op('oen', 'ONE')

    section(f'car {n}: service this floor - clear its requests, open the door')
    for f in range(FLOORS):
        op('ld', 'SERVICE'); op('and', f'{POS}+{f}')
        op('oen', 'RR', f'at floor {f}?')
        op('ld', 'ZERO')
        op('sto', f'{CAB}+{f}')
        for h in HALL[f]:
            op('sto', h)
    op('oen', 'SERVICE')
    op('ld', 'ONE'); op('sto', DWELL); op('sto', DWELL2); op('sto', DOOR)
    op('oen', 'ONE')

    section(f'car {n}: move one floor (the position is one-hot; shift it)')
    op('oen', 'MV_UP')
    for f in range(FLOORS - 2, -1, -1):
        op('ld', f'{POS}+{f}'); op('sto', f'{POS}+{f + 1}')
    op('ld', 'ZERO'); op('sto', f'{POS}+0')
    op('ld', 'ONE'); op('sto', DIR, 'remember: going up')
    op('oen', 'MV_DN')
    for f in range(1, FLOORS):
        op('ld', f'{POS}+{f}'); op('sto', f'{POS}+{f - 1}')
    op('ld', 'ZERO'); op('sto', f'{POS}+{FLOORS - 1}'); op('sto', DIR, 'remember: going down')
    op('oen', 'ONE')

    section(f'car {n}: direction lamps show what the car did on this tick')
    op('oen', 'TICK_EDGE')
    op('ld', 'MV_UP'); op('sto', LUP)
    op('ld', 'MV_DN'); op('sto', LDN)
    op('oen', 'ONE')


emit('/*')
emit('=' * 80)
emit('elevator6x2.asm:  6-floor, 2-car collective elevator controller for the TTLC')
emit('')
emit('Generated by gen_elevator6x2.py - edit that, not this.  I/O map: elevator6x2.h.')
emit('')
emit('Every scan: latch the buttons into the sticky request indicators, detect a')
emit('rising edge of the TICK input, then run both cars.  On a tick a car either')
emit('keeps its door open (two ticks), closes it, opens it because a request is')
emit('at its floor (clearing that floor\'s indicators), or moves one floor toward')
emit('the nearest pending request, continuing in its direction while there is')
emit('something ahead (collective control).  Both cars answer hall calls; car 1')
emit('has priority at a floor where both stand.')
emit('')
emit('asmsyntax=mc14500b')
emit('=' * 80)
emit('*/')
emit()
emit('include <elevator6x2.h>')
emit()
emit('loop:')
op('nopo', None, 'scan: outputs out, inputs in')

section('sticky request indicators: output n |= input n  (hall calls, both cabins)')
for n in range(22):
    op('ld', f'IN+{n}'); op('or', f'{n}'); op('sto', f'{n}')

section('tick edge: TICK_EDGE = TICK & !TICK_LAST')
op('ld', 'TICK'); op('andc', 'TICK_LAST'); op('sto', 'TICK_EDGE')
op('ld', 'TICK'); op('sto', 'TICK_LAST')

car(1)
car(2)

section('next scan')
op('nopf', None, 'back to address 0')
emit()
emit('// vim: sw=4 ts=4 et')
emit()

with open('elevator6x2.asm', 'w') as f:
    f.write('\n'.join(out))
print('wrote elevator6x2.asm (%d lines)' % len(out))
