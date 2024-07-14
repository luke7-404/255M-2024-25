#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
brain Brain_Timeout_timer;
controller Ctrl = controller(primary);

// Initialize motors and sensors as temporary names 
motor rightMid = motor(PORT3, ratio18_1, true); 
motor leftMid = motor(PORT8, ratio18_1, false); 
motor rightBack = motor(PORT12, ratio18_1, true); 
motor leftFront = motor(PORT13, ratio18_1, false); 
motor rightFront = motor(PORT18, ratio18_1, true); 
motor leftBack = motor(PORT20, ratio18_1, false); 
motor TEMP_MTR_NAME7 = motor(PORT1, ratio18_1, false); 
motor TEMP_MTR_NAME8 = motor(PORT2, ratio18_1, false); 

rotation leftTrack = rotation(PORT17, true);
rotation middleTrack = rotation(PORT11, false);
rotation rightTrack = rotation(PORT7, true);

inertial Inert = inertial(PORT19);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}