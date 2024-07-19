using namespace vex;

// Declare external devices
extern brain Brain;
extern brain Brain_Timeout_timer;
extern controller Ctrl;
extern motor rightMid;
extern motor leftMid;
extern motor rightBack;
extern motor leftBack;
extern motor rightFront;
extern motor leftFront;
extern motor TEMP_MTR_NAME7;
extern motor TEMP_MTR_NAME8;

extern rotation leftTrack;
extern rotation middleTrack;
extern rotation rightTrack;

extern inertial Inert;

// A external motor list that contains the available motors
extern vex::motor MotorList[]; 

extern sonar sonarRight;
extern sonar sonarBack;
extern sonar sonarLeft;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
