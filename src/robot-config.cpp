#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
brain Drivetrain_Timeout_timer;
//brain Motor_Timeout_timer;
controller Ctrl = controller(primary);

// Initialize motors and sensors as temporary names 
motor rightFront = motor(PORT16, ratio6_1, false);
motor rightMid = motor(PORT15, ratio6_1, false); 
motor rightBack = motor(PORT4, ratio6_1, false); 
motor leftFront = motor(PORT10, ratio6_1, true); 
motor leftMid = motor(PORT20, ratio6_1, true);
motor leftBack = motor(PORT19, ratio6_1, true); 

motor intake = motor(PORT21, ratio6_1, false); 
motor TEMP_MTR_NAME8 = motor(PORT21, ratio18_1, false); 

rotation XTrack = rotation(PORT7, false);
rotation YTrack = rotation(PORT8, false);

inertial Inert = inertial(PORT6);

vex::motor MotorList[] = {leftFront, leftMid, leftBack, rightFront, rightMid, rightBack, intake, TEMP_MTR_NAME8};

sonar sonarRight = sonar(Brain.ThreeWirePort.E);
sonar sonarBack = sonar(Brain.ThreeWirePort.C);
sonar sonarLeft = sonar(Brain.ThreeWirePort.A);

digital_out piz = digital_out(Brain.ThreeWirePort.A);