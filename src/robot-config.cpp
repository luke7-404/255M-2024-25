#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
brain Drivetrain_Timeout_timer;
controller Ctrl = controller(primary);


//! MOTORS
// Initialize motors 
motor rightFront = motor(PORT15, ratio6_1, false);
motor rightMid = motor(PORT16, ratio6_1, false); 
motor rightBack = motor(PORT3, ratio6_1, false); 
motor leftFront = motor(PORT10, ratio6_1, true); 
motor leftMid = motor(PORT20, ratio6_1, true);
motor leftBack = motor(PORT19, ratio6_1, true); 

motor intakeFirst = motor(PORT14, true); 
motor intakeSecond = motor(PORT11, false);  

// Tether the two intake motors together 
motor_group intake = motor_group(intakeFirst, intakeSecond);

motor ladyBrown = motor(PORT12, false);

// A external motor list that contains the available motors
vex::motor MotorList[] = {leftFront, leftMid, leftBack, rightFront, rightMid, rightBack, intakeSecond, ladyBrown};


//! V5 Sensors
rotation ladyLeftRotation = rotation(PORT9, false);
rotation ladyRightRotation = rotation(PORT13, false);

rotation XTrack = rotation(PORT7, false);
rotation YTrack = rotation(PORT8, false);

inertial Inert = inertial(PORT6);

distance clawDistance = distance(PORT18);

triport sonarSensors = triport(PORT5); // Triport extender


//! Triport Sensors
sonar sonarLeft = sonar(sonarSensors.G);
sonar sonarBack = sonar(sonarSensors.E);
sonar sonarRight = sonar(sonarSensors.A);

// Pneumatics
digital_out intakeRaiser = digital_out(Brain.ThreeWirePort.A);
digital_out mogoClaw = digital_out(Brain.ThreeWirePort.B);
digital_out doinker = digital_out(Brain.ThreeWirePort.C);
