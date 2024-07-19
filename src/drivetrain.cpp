#include "drivetrain.hpp"

Odom_Data odom; // Object to get odometry data
double targetX = odom.xPosGlobal; // Target X to drive to
double targetY = odom.yPosGlobal; // Target Y to drive to
double targetAngle = 0; // Target angle to face


// Controls if the feedback loop runs or not
bool runControl = false;

// After the timeout value is reached, the robot won't move
int timeOutValue = 2500;


/*
NOTES

Bottom left corner of field is (0,0)

Angles are like a unit circle, so the positive X direction is 0 and positive Y direction is pi/2

*/

void driveTo(double Target_X, double Target_Y, double Target_Angle, double timeOutLength) {
  targetX = Target_X; // Set target X
  targetY = Target_Y; // Set target Y
  targetAngle = Target_Angle * M_PI / 180; // convert deg to rad

  timeOutValue = timeOutLength; // Set timeout
  runControl = true; // Let the feedback loop move the drivetrain
  Brain_Timeout_timer.resetTimer(); // Reset and start countdown
}


void turnTo(double Target_Angle, double timeOutLength) {
  targetAngle = Target_Angle * M_PI / 180; // convert deg to rad

  timeOutValue = timeOutLength; // Set timeout

  // Set target X and Y as current position
  targetX = odom.xPosGlobal;
  targetY = odom.yPosGlobal;

  runControl = true; // Let the feedback loop move the drivetrain
  Brain_Timeout_timer.resetTimer(); // Reset and start countdown
}

void turnToPoint(double xCoordToFace, double yCoordToFace, double timeOutLength) {
  
  // calculate angle
  targetAngle = atan2(yCoordToFace - odom.yPosGlobal, xCoordToFace - odom.xPosGlobal);
  
  /* if angle is negative, then the angle is adjusted to be a positive angle 
     equivalent to the same angle in the range of 0 to 2pi. */
  if(targetAngle < 0) {
    targetAngle = 2 * M_PI - fabs(targetAngle);
  }

  // Set X and Y as current position
  targetX = odom.xPosGlobal;
  targetY = odom.yPosGlobal;

  timeOutValue = timeOutLength; // Set timeout

  runControl = true; // Let the feedback loop move the drivetrain
  Brain_Timeout_timer.resetTimer(); // Reset and start countdown
}



double PID_Data::error = 0; // Target distance difference
double PID_Data::prevError = 0; // Last cycle distance difference
const double maxError = 0.75; // Error tolerance 
    // 0.5

double PID_Data::integral = 0; // The accumulated error
const double integralBound = 30; // The minium distance to the target to use the integral
    // 1.5

double PID_Data::derivative = 0; // The change in error between cycles (prev and current)

const double kP = 0.059; // Error Gain 
    // 0.07 // 0.025 // 0.03 // 0.05 // 0.055
const double kI = 0.008; // Integral Gain 
    // 0.007 //0.006 // 0.005
const double kD = 0.255; // Derivative Gain
    // 0.40 // 0.20 //0.25

double PID_Data::drivePowerPID = 0; // The power that will be sent to the motors

void drivePID() {
  PID_Data::prevError = PID_Data::error; // update the previous error

  //Error is equal to the total distance away from the target (uses distance formula with current position and target location)
  PID_Data::error = sqrt(pow((odom.xPosGlobal - targetX), 2) + pow((odom.yPosGlobal - targetY), 2));
  //PID_Data::error = targetX - odom.xPosGlobal;

  //only use integral if close enough to target but allows for tolerance
  if(fabs(PID_Data::error) < integralBound && fabs(PID_Data::error) > maxError) {
    PID_Data::integral += PID_Data::error; // Add the error to the integral
  } else PID_Data::integral = 0; // Reset the integral if we are too far from the target

  PID_Data::derivative = PID_Data::error - PID_Data::prevError; // calculate the derivative

  // calculate the power output
  PID_Data::drivePowerPID = (PID_Data::error * kP + PID_Data::integral * kI + PID_Data::derivative * kD);

  //Limit power output to 12V
  if(PID_Data::drivePowerPID > 12) PID_Data::drivePowerPID = 12;

  // if we are in the tolerance, stop driving
  if(fabs(PID_Data::error) < maxError) PID_Data::drivePowerPID = 0;
}

double PID_Data::turn_Error = 0; // Target angle difference
double PID_Data::turn_PrevError = 0; // Last cycle angle difference
const double turn_MaxError =  0.010472; // tolerance of 0.6 degrees (in rads)
    //0.01309; // 0.75 deg //0.01745329 // 1 deg //0.00087266 // 0.05 deg

double PID_Data::turn_Integral = 0; // The accumulated error
const double turn_IntegralBound = 0.34906585; // Minimum angle to target to use the integral (20 degrees)
    // 0.17453293; // 10 degrees

double PID_Data::turn_Derivative = 0; // The change in error between cycles (prev and current)

const double turn_kP = 4.75; // Turn Error Gain 
    // 1.50 // 1.25 // 1.125 // 1.0625 // 1 // 2
const double turn_kI = 0.45; // Turn Integral Gain 
    // 0.3 //0.1
const double turn_kD = 1.0; // Turn Derivative Gain
    // 0.45; //0.5

bool toLeft = false; // Logic to determine which direction to turn
double PID_Data::turnPowerPID = 0; // The power that will be sent to the motors

void turnPID() {
  PID_Data::turn_PrevError = PID_Data::turn_Error; // update the previous error

  // Determine the error based on the direction we are turning
  // If turning left, the error is the difference between the current facing direction and the target direction
  if (toLeft){
    // Error is the difference between the current facing direction and the target direction
    PID_Data::turn_Error = odom.currentAbsoluteOrientation - targetAngle;
  }
  // If turning right, the error is the difference between the target direction and the current facing direction
  else {
    // Error is the difference between the target direction and the current facing direction
    PID_Data::turn_Error = targetAngle - odom.currentAbsoluteOrientation;
  }

  //PID_Data::turn_Error = currentAbsoluteOrientation - targetAngle;
  
  // If the error is greater than pi radians, wrap it around to the other side of the circle
  if(fabs(PID_Data::turn_Error) > M_PI) {
    PID_Data::turn_Error = fabs(2 * M_PI - PID_Data::turn_Error);
    // Flip the toLeft flag so that the robot continues turning in the same direction
    toLeft = true;
  }

  // only use integral if close enough to target but allows for tolerance
  if(fabs(PID_Data::turn_Error) < turn_IntegralBound && fabs(PID_Data::turn_Error) > turn_MaxError) {
    PID_Data::turn_Integral += PID_Data::turn_Error; // Add the error to the integral
  } else PID_Data::turn_Integral = 0; // Reset the integral if we are too far from the target

  // calculate the derivative
  PID_Data::turn_Derivative = PID_Data::turn_Error - PID_Data::turn_PrevError;

  // calculate the power output
  PID_Data::turnPowerPID = (PID_Data::turn_Error * turn_kP + 
                             PID_Data::turn_Integral * turn_kI + 
                             PID_Data::turn_Derivative * turn_kD);

  //Limit power output to 12V
  if(PID_Data::turnPowerPID > 12) PID_Data::turnPowerPID = 12;

  // If in tolerance range, cut power
  if(fabs(PID_Data::turn_Error) < turn_MaxError) PID_Data::turnPowerPID = 0;

  // If the input angle is negative, negate power
  if (toLeft) PID_Data::turnPowerPID *= -1;
}


int driveControl() {

  //loop to constantly execute commands and update values
  while(1) {
    
    //Check if we should run the control
    if(runControl) {
      //xDistToTarget = xTargetLocation - xPosGlobal;
      //yDistToTarget = yTargetLocation - yPosGlobal;
      odom.xPosGlobal = (leftTrack.position(deg) + rightTrack.position(deg)) / 2;
      odom.currentAbsoluteOrientation = (360 + Inert.heading(rotationUnits::deg)) * M_PI / 180.0;

      //get PID values for driving and turning
      drivePID();
      turnPID();

      //set power for each motor
      leftFront.spin(fwd, PID_Data::drivePowerPID + PID_Data::turnPowerPID, volt);
      leftMid.spin(fwd, PID_Data::drivePowerPID + PID_Data::turnPowerPID, volt);
      leftBack.spin(fwd, PID_Data::drivePowerPID + PID_Data::turnPowerPID, volt);
      rightBack.spin(fwd, PID_Data::drivePowerPID - PID_Data::turnPowerPID, volt);
      rightMid.spin(fwd, PID_Data::drivePowerPID - PID_Data::turnPowerPID, volt);
      rightFront.spin(fwd, PID_Data::drivePowerPID - PID_Data::turnPowerPID, volt);

      //Check if we should stop
      if(fabs(PID_Data::error) < 0.1 && fabs(PID_Data::turn_Error) < 0.003) {
        runControl = false;
      }

      // If the time out is reached, stop the loop
      if(Brain_Timeout_timer.timer(timeUnits::msec) > timeOutValue) {
        runControl = false;
      }

    }
    //What to do when not using the feedback loop
    else {
      leftFront.stop(coast);
      leftMid.stop(coast);
      leftBack.stop(coast);
      rightBack.stop(coast);
      rightMid.stop(coast);
      rightFront.stop(coast);
    }
    
    // Set cycle time
    task::sleep(20);
  }
  return 1;
}



void calibrate_IMU(){
  Inert.calibrate();

  // indicate calibrating on the controller screen
  while (Inert.isCalibrating()){
    Ctrl.Screen.clearLine(3); // clear line
    Ctrl.Screen.print("Calibrating"); // print calibrating
    task::sleep(100);
  } 
  
  Inert.setHeading(270, degrees);

  // when finished it prints done on the controller screen
  Ctrl.Screen.clearLine(3); // clear line on controller 

  // Print on brain and controller Screen done
  Ctrl.Screen.print("Done");

  // Vibrates the controller
  Ctrl.rumble("..");

}

void reset_Tracking_Wheels(){
  leftTrack.resetPosition();
  middleTrack.resetPosition();
  rightTrack.resetPosition();
  std::cout << "Tracking Wheels reset" << std::endl;
}

std::vector<double> get_Motor_Data(){

  // create a temp vector to store the data
  std::vector<double> data = {0, 0, 0, 0, 0};

  /*
    fahrenheit = index 0
    volt = index 1
    amp = index 2
    Nm = index 3
    efficiency = index 4
  */

  // get data from each motor
  for(uint8_t i = 0; i < 6; i++){
    data[0] += MotorList[i].temperature(fahrenheit);
    data[1] += MotorList[i].voltage(volt);
    data[2] += MotorList[i].current(amp);
    data[3] += MotorList[i].torque(Nm);
    data[4] += MotorList[i].efficiency();
  }
  
  // take the average
  data[0] /= 6.0;
  data[1] /= 6.0;
  data[2] /= 6.0;
  data[3] /= 6.0;
  data[4] /= 6.0;

  return data; // return the vector
}

double radToDeg(double rad){
  rad = rad * 180 / M_PI;
  return rad;
}

PID_Data::PID_Data(){
  std::cout << "Drivetrain object created" << std::endl;
}

PID_Data::~PID_Data(){
  std::cout << "Drivetrain object destroyed" << std::endl;
}