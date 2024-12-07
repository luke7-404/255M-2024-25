using namespace vex;

// Declare external devices \\

extern brain Brain;
extern brain Drivetrain_Timeout_timer;
extern controller Ctrl;

//! MOTORS
extern motor rightMid;
extern motor leftMid;
extern motor rightBack;
extern motor leftBack;
extern motor rightFront;
extern motor leftFront;
extern motor intakeFirst;
extern motor intakeSecond;
extern motor ladyBrown;

// A external motor list that contains the available motors
extern motor MotorList[]; 

// Tether the two intake motors together 
extern motor_group intake;

//! V5 Sensors
extern rotation ladyLeftRotation;
extern rotation ladyRightRotation;
extern rotation YTrack;
extern rotation XTrack;
extern inertial Inert;
extern distance clawDistance;

extern triport sonarSensors; // Triport extender

//! Triport Sensors
extern sonar sonarRight;
extern sonar sonarBack;
extern sonar sonarLeft;

// Pneumatics

extern digital_out intakeRaiser;
extern digital_out mogoClaw;
extern digital_out doinker;

