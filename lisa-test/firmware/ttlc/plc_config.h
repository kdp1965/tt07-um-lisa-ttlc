// Define our inputs
#define   RR        136       // The Result Reg as an input
#define   UP1       48        // UP button on floor 1
#define   UP2       UP1+1     // UP button on floor 2
#define   UP3       UP2+1     // UP button on floor 3
#define   DOWN2     UP3+1     // DOWN button on floor 2
#define   DOWN3     DOWN2+1   // DOWN button on floor 3
#define   DOWN4     DOWN3+1   // DOWN button on floor 4

#define   FLOOR1    7         // Request for floor 1
#define   FLOOR2    8         // Request for floor 1
#define   FLOOR3    9         // Request for floor 1
#define   FLOOR4    10        // Request for floor 1

#define   OPEN      11        // Open the door
#define   CLOSE     12        // Close the door
#define   ALARM     13        // Alarm


// Define out outputs
#define   UP1_LED       0     // UP button on floor 1
#define   UP2_LED       1     // UP button on floor 2
#define   UP3_LED       2     // UP button on floor 3
#define   DOWN2_LED     4     // DOWN button on floor 2
#define   DOWN3_LED     5     // DOWN button on floor 3
#define   DOWN4_LED     6     // DOWN button on floor 4
#define   FLOOR1_LED    7     // Request for floor 1
#define   FLOOR2_LED    8     // Request for floor 1
#define   FLOOR3_LED    9     // Request for floor 1
#define   FLOOR4_LED    10    // Request for floor 1
#define   OPEN_LED      11    // Open the door
#define   CLOSE_LED     12    // Close the door
#define   ALARM_LED     13    // Alarm

#define   CLOSE_DOOR    14
#define   OPEN_DOOR     15


// Define temp storage locations
enum {
  UP1_HOLD  =  96,
  UP2_HOLD,
  UP3_HOLD,
  DOWN2_HOLD,
  DOWN3_HOLD,
  DOWN4_HOLD,
  FLOOR1_HOLD,
  FLOOR2_HOLD,
  FLOOR3_HOLD,
  FLOOR4_HOLD,
  TIMER1_ACTIVE,
  TIMER2_ACTIVE
};

