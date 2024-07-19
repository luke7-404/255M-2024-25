#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
brain Brain_Timeout_timer;
controller Ctrl = controller(primary);

// Initialize motors and sensors as temporary names 
motor rightFront = motor(PORT10, ratio18_1, false);
motor rightMid = motor(PORT20, ratio18_1, false); 
motor rightBack = motor(PORT19, ratio18_1, false); 
motor leftFront = motor(PORT3, ratio18_1, true); 
motor leftMid = motor(PORT11, ratio18_1, true);
motor leftBack = motor(PORT12, ratio18_1, true); 

motor TEMP_MTR_NAME7 = motor(PORT21, ratio18_1, false); 
motor TEMP_MTR_NAME8 = motor(PORT21, ratio18_1, false); 

rotation leftTrack = rotation(PORT13, true);
rotation middleTrack = rotation(PORT9, false);
rotation rightTrack = rotation(PORT18, true);

inertial Inert = inertial(PORT17);

vex::motor MotorList[] = {leftFront, leftMid, leftBack, rightFront, rightMid, rightBack, TEMP_MTR_NAME7, TEMP_MTR_NAME8};

sonar sonarRight = sonar(Brain.ThreeWirePort.E);
sonar sonarBack = sonar(Brain.ThreeWirePort.C);
sonar sonarLeft = sonar(Brain.ThreeWirePort.A);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}