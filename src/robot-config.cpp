#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// Initialize motors and sensors as temporary names 
motor TestMotor1 = motor(PORT1, ratio18_1, false); 
motor TestMotor2 = motor(PORT2, ratio18_1, false); 
motor TestMotor3 = motor(PORT3, ratio18_1, false); 
motor TestMotor4 = motor(PORT4, ratio18_1, false); 
motor TestMotor5 = motor(PORT5, ratio18_1, false); 
motor TestMotor6 = motor(PORT6, ratio18_1, false); 
motor TestMotor7 = motor(PORT7, ratio18_1, false); 
motor TestMotor8 = motor(PORT20, ratio18_1, false); 

rotation leftTrack = rotation(PORT10, false);
rotation middleTrack = rotation(PORT11, false);
rotation rightTrack = rotation(PORT12, false);

inertial Inertial = inertial(PORT14);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}