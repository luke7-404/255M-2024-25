#include "drivetrain.hpp"

Odom_Data odom; // Object to get odometry data
competition EventCheck;
double targetX = odom.xPosGlobal; // Target X to drive to
double targetY = odom.yPosGlobal; // Target Y to drive to
double targetAngle = odom.currentAbsoluteOrientation; // Target angle to face


// Controls if the feedback loop runs or not
bool runControl = false;

// After the timeout value is reached, the robot won't move
int timeOutValue = 2500;


/*
NOTES

Bottom left corner of field is (0,0)

Angles are like a unit circle, so the positive X direction is 0 and positive Y direction is pi/2

*/
//double nextAngle = 0;


void driveToPoint(double Target_X, double Target_Y, double timeOutLength) {
  targetX = Target_X; // Set target X
  targetY = Target_Y; // Set target Y

  targetAngle = atan2(Target_X-odom.xPosGlobal, Target_Y-odom.yPosGlobal);
  

  /*if(fabs(targetAngle) > M_PI) {
    targetAngle = -fabs(targetAngle)/targetAngle * (M_PI * 2) - fabs(targetAngle);
  }*/

  timeOutValue = timeOutLength; // Set timeout
  runControl = true; // Let the feedback loop move the drivetrain
  Brain_Timeout_timer.resetTimer(); // Reset and start countdown
}


void turnToAngle(double Target_Angle, double timeOutLength) {
  if (targetAngle < 0) targetAngle += 360;
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
  //targetAngle = atan2(xCoordToFace - odom.xPosGlobal,yCoordToFace - odom.yPosGlobal) - odom.currentAbsoluteOrientation;
  //atan2(yCoordToFace - odom.yPosGlobal, xCoordToFace - odom.xPosGlobal);
  //atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading())
  //targetAngle = atan2(odom.xPosGlobal- xCoordToFace, odom.yPosGlobal - yCoordToFace) - odom.currentAbsoluteOrientation - M_PI_2;
  
  targetAngle = atan2(xCoordToFace-odom.xPosGlobal, yCoordToFace-odom.yPosGlobal);

  /* if angle is negative, then the angle is adjusted to be a positive angle 
     equivalent to the same angle in the range of 0 to 2pi. 
  if(targetAngle < 0) {
    targetAngle = 2 * M_PI - fabs(targetAngle);
  }*/

  if(fabs(targetAngle) > M_PI) {
    targetAngle = -fabs(targetAngle)/targetAngle * (M_PI * 2) - fabs(targetAngle);
  }

  // Set X and Y as current position
  targetX = odom.xPosGlobal;
  targetY = odom.yPosGlobal;
  //toggleClamp = true;
  timeOutValue = timeOutLength; // Set timeout

  runControl = true; // Let the feedback loop move the drivetrain
  Brain_Timeout_timer.resetTimer(); // Reset and start countdown
}

// If the input voltage is higher or lower than 
double clamp(double input, double min, double max){
  if( input > max ) return(max);
  if(input < min) return(min);
  return(input);
}


double PID_Data::error = 0; // Target distance difference
double PID_Data::prevError = 0; // Last cycle distance difference
const double maxError = 0.75;//2/*5/*0.75*/; // Error tolerance 
    // 0.5

double PID_Data::integral = 0; // The accumulated error
const double integralBound = 15; // The minium distance to the target to use the integral
    // 1.5

double PID_Data::derivative = 0; // The change in error between cycles (prev and current)

const double kP = 0.1;//0.5; // Error Gain 
    // 0.059// 0.07 // 0.025 // 0.03 // 0.05 // 0.055
const double kI = 0.0;//0.0045; // Integral Gain 
    // 0.008// 0.007 //0.006 // 0.005
const double kD = 0;//0.255; // Derivative Gain
    // 0.255 // 0.40 // 0.20 //0.25

double PID_Data::drivePowerPID = 0; // The power that will be sent to the motors



double PID_Data::turn_Error = 0; // Target angle difference
double PID_Data::turn_PrevError = 0; // Last cycle angle difference
const double turn_MaxError =  0.010472; // tolerance of 0.6 degrees (in rads)
    //0.01309; // 0.75 deg //0.01745329 // 1 deg //0.00087266 // 0.05 deg

double PID_Data::turn_Integral = 0; // The accumulated error
const double turn_IntegralBound = 0.34906585; // Minimum angle to target to use the integral (20 degrees)
    // 0.17453293; // 10 degrees

double PID_Data::turn_Derivative = 0; // The change in error between cycles (prev and current)

const double turn_kP = 0.1;//4.75; // Turn Error Gain 
    // 1.50 // 1.25 // 1.125 // 1.0625 // 1 // 2
const double turn_kI = 0;//0.45; // Turn Integral Gain 
    // 0.3 //0.1
const double turn_kD = 0;//1.0; // Turn Derivative Gain
    // 0.45; //0.5

double PID_Data::turnPowerPID = 0; // The power that will be sent to the motors

void PID(){

  // update the previous error
  PID_Data::prevError = PID_Data::error;
  PID_Data::turn_PrevError = PID_Data::turn_Error; 

  //Error is equal to the total distance away from the target
  PID_Data::error = sqrt(pow((targetX - odom.xPosGlobal), 2) + pow((targetY - odom.yPosGlobal), 2));
  //PID_Data::error = (targetX - odom.xPosGlobal) + (targetY - odom.yPosGlobal);
  PID_Data::turn_Error = targetAngle - odom.currentAbsoluteOrientation;

  //std::cout << "turn error: " << PID_Data::turn_Error << std::endl;

  // If the error is greater than pi radians, wrap it around to the other side of the circle
  /*if(fabs(PID_Data::turn_Error) > M_PI) {
    PID_Data::turn_Error = -fabs(PID_Data::turn_Error)/PID_Data::turn_Error * (M_PI * 2) - fabs(PID_Data::turn_Error);
  }*/

  //only use integral if close enough to target but allows for tolerance
  /*if(fabs(PID_Data::error) < integralBound && fabs(PID_Data::error) > maxError) {
    PID_Data::integral += PID_Data::error; // Add the error to the integral
  } else PID_Data::integral = 0; // Reset the integral if we are too far from the target


  // only use integral if close enough to target but allows for tolerance
  if(fabs(PID_Data::turn_Error) < turn_IntegralBound && fabs(PID_Data::turn_Error) > turn_MaxError) {
    PID_Data::turn_Integral += PID_Data::turn_Error; // Add the error to the integral
  } else PID_Data::turn_Integral = 0; // Reset the integral if we are too far from the target

  PID_Data::derivative = PID_Data::error - PID_Data::prevError; // calculate the derivative

  // calculate the derivative
  PID_Data::turn_Derivative = PID_Data::turn_Error - PID_Data::turn_PrevError;*/

  // calculate the power output
  PID_Data::drivePowerPID = (PID_Data::error * kP + PID_Data::integral * kI + PID_Data::derivative * kD);
  PID_Data::turnPowerPID = (PID_Data::turn_Error * turn_kP + 
                            PID_Data::turn_Integral * turn_kI + 
                            PID_Data::turn_Derivative * turn_kD);

  //Limit power output to 12V
  PID_Data::drivePowerPID = clamp(PID_Data::drivePowerPID, -12, 12);
  PID_Data::turnPowerPID = clamp(PID_Data::turnPowerPID, -12, 12);

  // If in tolerance range, cut power
  if(fabs(PID_Data::error) < maxError) PID_Data::drivePowerPID = 0;
  if(fabs(PID_Data::turn_Error) < turn_MaxError) PID_Data::turnPowerPID = 0;
}

int driveControl() {

  //loop to constantly execute commands and update values
  while(1) {
    
    //Check if we should run the control
    if(runControl) {

      //get PID values for driving and turning
      PID();

      //set power for each motor
      leftFront.spin(fwd, PID_Data::drivePowerPID + PID_Data::turnPowerPID, volt);
      leftMid.spin(fwd, PID_Data::drivePowerPID + PID_Data::turnPowerPID, volt);
      leftBack.spin(fwd, PID_Data::drivePowerPID + PID_Data::turnPowerPID, volt);
      rightBack.spin(fwd, PID_Data::drivePowerPID - PID_Data::turnPowerPID, volt);
      rightMid.spin(fwd, PID_Data::drivePowerPID - PID_Data::turnPowerPID, volt);
      rightFront.spin(fwd, PID_Data::drivePowerPID - PID_Data::turnPowerPID, volt);

      // If the time out is reached, stop the loop
      if(Brain_Timeout_timer.timer(timeUnits::msec) > timeOutValue) {
        runControl = false;
      }

    }else if(EventCheck.isAutonomous()){
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
  
  Inert.setHeading(90, degrees);

  // when finished it prints done on the controller screen
  Ctrl.Screen.clearLine(3); // clear line on controller 

  // Print on brain and controller Screen done
  Ctrl.Screen.print("Done");

  // Vibrates the controller
  Ctrl.rumble("..");

}

void reset_Tracking_Wheels(){
  XTrack.resetPosition();
  YTrack.resetPosition();
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
  for (auto &&dat : data){
    dat /= 6.0;
    /*data[1] /= 6.0;
    data[2] /= 6.0;
    data[3] /= 6.0;
    data[4] /= 6.0; */
  }

  return data; // return the vector
}

double radToDeg(double rad){
  return rad * 180 / M_PI;
}

double ControlSense(double percent, int selectedCurve){
  percent = clamp(percent, -127, 127);
  selectedCurve = clamp(selectedCurve, 0, 3);
  
  switch (selectedCurve) {
    
    // Exp curve 1
    // Positive input: f(x) = 1.2((1.043)^x) - 1.2 + 0.2*x
    // Negative input: -f(-x)
    case 1:

      if(percent >= 0) return 1.2*pow(1.043, percent) - 1.2 + 0.2*percent;
      else return -(1.2*pow(1.043, -percent) - 1.2 + 0.2* -percent);

    break;

    // Exp curve 2
    // Positive input: h(x) = 0.5(2)^(20x-10) 
    // Negative input: -h(-x) 
    case 2:

      if (percent >= 0) return 0.5*pow(2, (20*percent)-10);
      else return -(0.5*pow(2, (20*-percent)-10));

    break;

    // Quad curve
    // Positive input: g(x) = 2x^2
    // Negative input: g(x) = 1-((-2x+2)^2)/2
    case 3:
      if (percent >= 0) return 2 * pow(percent, 2);
      else return 1- pow(-2*percent + 2, 2) /2;  
    break;
  
    default:
      return percent;
    break;
  }

  return 0.0;
}

void joeySticks(int32_t forward, int32_t turn){
  // Motor voltage on a range of -12 to 12
  double leftVolt; // Declare left side voltage 
  double rightVolt; // Declare right side voltage

  // Calculate voltage
  leftVolt = clamp(12 * (ControlSense(forward + turn, 3) / 100.0), -12.0, 12.0);
  rightVolt = clamp(12 * (ControlSense(forward - turn, 3) / 100.0), -12.0, 12.0);

  // Apply leftVolt to the left motors
  leftFront.spin(fwd, leftVolt, volt);
  leftMid.spin(fwd, leftVolt, volt);
  leftBack.spin(fwd, leftVolt, volt);
  // Apply rightVolt to the right motors
  rightBack.spin(fwd, rightVolt, volt);
  rightMid.spin(fwd, rightVolt, volt);
  rightFront.spin(fwd, rightVolt, volt);

}