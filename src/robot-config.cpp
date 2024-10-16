#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
brain Brain_Timeout_timer;
controller Ctrl = controller(primary);

// Initialize motors and sensors as temporary names 
motor rightFront = motor(PORT5, ratio18_1, false);
motor rightMid = motor(PORT19, ratio18_1, false); 
motor rightBack = motor(PORT9, ratio18_1, false); 
motor leftFront = motor(PORT6, ratio18_1, true); 
motor leftMid = motor(PORT8, ratio18_1, true);
motor leftBack = motor(PORT7, ratio18_1, true); 

motor intake = motor(PORT4, ratio6_1, false); 
motor TEMP_MTR_NAME8 = motor(PORT21, ratio18_1, false); 

rotation XTrack = rotation(PORT16, false);
rotation YTrack = rotation(PORT20, true);

inertial Inert = inertial(PORT10, turnType::right);

vex::motor MotorList[] = {leftFront, leftMid, leftBack, rightFront, rightMid, rightBack, intake, TEMP_MTR_NAME8};

sonar sonarRight = sonar(Brain.ThreeWirePort.E);
sonar sonarBack = sonar(Brain.ThreeWirePort.C);
sonar sonarLeft = sonar(Brain.ThreeWirePort.A);

digital_out piz = digital_out(Brain.ThreeWirePort.A);
digital_out speaker = digital_out(Brain.ThreeWirePort.H);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}