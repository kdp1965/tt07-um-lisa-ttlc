// elevator6x2.h - I/O map of the 6-floor, 2-car elevator controller
//
// Inputs are momentary buttons (TTLC address 48 + n); the matching output
// (same number) is the sticky request indicator, lit until a car serves it.
//
//   inputs 0-9    hall calls      0: F0 up   1: F1 up  2: F1 down  3: F2 up  4: F2 down
//                                 5: F3 up   6: F3 down  7: F4 up  8: F4 down  9: F5 down
//   inputs 10-15  car 1 cabin buttons, floors 0-5
//   inputs 16-21  car 2 cabin buttons, floors 0-5
//   input  47     TICK: a clock from the I/O rack; cars move one floor per tick
//
//   outputs 0-21  request indicators (mirror the inputs above)
//   outputs 22/23 car 1 / car 2 door open
//   outputs 24/25 car 1 moving up / down     26/27  car 2 moving up / down
//   outputs 32-37 car 1 is at floor 0-5      38-43  car 2 is at floor 0-5

// MC14500B constants
#define   RR          136     // the result register, readable as data
#define   ZERO        137     // reads as 0
#define   ONE         138     // reads as 1

// inputs (buttons)
#define   IN          48
#define   BTN_UP0     IN+0
#define   BTN_UP1     IN+1
#define   BTN_DN1     IN+2
#define   BTN_UP2     IN+3
#define   BTN_DN2     IN+4
#define   BTN_UP3     IN+5
#define   BTN_DN3     IN+6
#define   BTN_UP4     IN+7
#define   BTN_DN4     IN+8
#define   BTN_DN5     IN+9
#define   BTN_CAB1    IN+10    // + floor
#define   BTN_CAB2    IN+16    // + floor
#define   TICK        IN+47

// outputs (indicators and car state)
#define   HALL_UP0    0
#define   HALL_UP1    1
#define   HALL_DN1    2
#define   HALL_UP2    3
#define   HALL_DN2    4
#define   HALL_UP3    5
#define   HALL_DN3    6
#define   HALL_UP4    7
#define   HALL_DN4    8
#define   HALL_DN5    9
#define   CAB1        10      // + floor: car 1 cabin request indicators
#define   CAB2        16      // + floor: car 2
#define   DOOR1       22
#define   DOOR2       23
#define   UP1         24
#define   DOWN1       25
#define   UP2         26
#define   DOWN2       27
#define   POS1        32      // + floor: car 1 position (one-hot)
#define   POS2        38      // + floor: car 2 position (one-hot)

// storage (96-127; 104 is skipped, it is the LISA interrupt)
enum {
  TICK_LAST = 96,          // tick input on the previous scan
  TICK_EDGE,               // tick just went high: the cars act on this scan
  R0,                      // requests that concern the car being worked on, per floor
  R1,
  R2,
  R3,
  R4,
  R5,
  ANY_ABOVE = 105,         // ...above / below / at the car's floor
  ANY_BELOW,
  REQ_HERE,
  SERVICE,                 // this tick: open the door and clear this floor's requests
  MV_UP,                   // this tick: move up / down
  MV_DN,
  STAY,                    // this tick: first dwell tick, keep the door open
  CLOSING,                 // this tick: second dwell tick, close the door
  C1_DIR,                  // per car: last direction (1 = up)
  C1_DWELL,                //   door open
  C1_DWELL2,               //   first dwell tick still pending
  C2_DIR,
  C2_DWELL,
  C2_DWELL2
};
