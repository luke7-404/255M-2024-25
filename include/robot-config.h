using namespace vex;

// Declare external devices
extern brain Brain;
extern brain Drivetrain_Timeout_timer;
extern brain Motor_Timeout_timer;
extern controller Ctrl;
extern motor rightMid;
extern motor leftMid;
extern motor rightBack;
extern motor leftBack;
extern motor rightFront;
extern motor leftFront;
extern motor intake;
extern motor TEMP_MTR_NAME8;

extern rotation YTrack;
extern rotation XTrack;

extern inertial Inert;

// A external motor list that contains the available motors
extern vex::motor MotorList[]; 

extern sonar sonarRight;
extern sonar sonarBack;
extern sonar sonarLeft;

extern digital_out piz;

extern digital_out speaker;
