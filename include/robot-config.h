using namespace vex;

// Declare external devices
extern brain Brain;
extern motor TestMotor1;
extern motor TestMotor2;
extern motor TestMotor3;
extern motor TestMotor4;
extern motor TestMotor5;
extern motor TestMotor6;
extern motor TestMotor7;
extern motor TestMotor8;

extern rotation leftTrack;
extern rotation middleTrack;
extern rotation rightTrack;

extern inertial Inertial;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
