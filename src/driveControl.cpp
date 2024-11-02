#include "drivetrain.hpp"

// Competition object to keep track of time events
competition EventCheck;

//! ODOM

//* CONSTANTS */

// The ratio between the angle and distance traveled by the wheels
const double in_to_deg_ratio = M_PI*(2.75/360.0);

//The starting x and y coordinates of the bot (INCHES)
  //* These distances are relative to bottom left corner (0,0) on the field
const double X_START = 72; //x_distance;
const double Y_START = 24; //sensorY;

//Distances of tracking wheels from robot's center (INCHES)
const double YTrackDist = 2.25;//1.7705;
const double XTrackDist = 2.25; //2.3315;


//* Calculated Values (every loop)
//Angles (DEGREES) 
  //! Is converted to radians for math
double Odom_Data::currentX = 0;
double Odom_Data::currentY = 0;

//The previous X and Y position of the bot
double Odom_Data::previousX = 0;
double Odom_Data::previousY = 0;

//Distances traveled by tracking wheels each loop (INCHES)
double Odom_Data::deltaXTrack = 0;
double Odom_Data::deltaYTrack = 0;

//The current angle of the bot (RADIANS)
double Odom_Data::currentTheta = 0;

//The previous angle of the bot (RADIANS)
double Odom_Data::previousTheta = 0;

//The change in Theta each loop (RADIANS)
double Odom_Data::deltaTheta = 0;

//The changes in the robot's X and Y positions (INCHES)
double Odom_Data::XLocal = 0;
double Odom_Data::YLocal = 0;

//The robot's position relative to the field
double Odom_Data::local_polar_angle = 0;
double Odom_Data::local_polar_position = 0;

// The global angle of the bot relative to field (RADIANS)
double Odom_Data::global_polar_angle = 0;

//The global position of the bot relative to field (INCHES)
double Odom_Data::xPosGlobal = X_START;
double Odom_Data::yPosGlobal = Y_START;



int positionTracking() {
  while(1) {

    Odom_Data::currentX = XTrack.position(rotationUnits::deg) * in_to_deg_ratio;
    Odom_Data::currentY = YTrack.position(rotationUnits::deg) * in_to_deg_ratio;

    Odom_Data::deltaXTrack = Odom_Data::currentX-Odom_Data::previousX;
    Odom_Data::deltaYTrack = Odom_Data::currentY-Odom_Data::previousY;

    Odom_Data::previousX = Odom_Data::currentX;
    Odom_Data::previousY = Odom_Data::currentY;

    
    Odom_Data::currentTheta = degToRad(reduce_0_to_360( Inert.rotation()/* *360.0/gyro_scale*/ ));
    Odom_Data::deltaTheta = Odom_Data::currentTheta-Odom_Data::previousTheta;
    
    Odom_Data::previousTheta = Odom_Data::currentTheta;

    if (Odom_Data::deltaTheta == 0) {
        Odom_Data::XLocal = Odom_Data::deltaXTrack;
        Odom_Data::YLocal = Odom_Data::deltaYTrack;
    } else {
        Odom_Data::XLocal = (2*sin(Odom_Data::deltaTheta/2))*((Odom_Data::deltaXTrack/Odom_Data::deltaTheta)+XTrackDist); 
        Odom_Data::YLocal = (2*sin(Odom_Data::deltaTheta/2))*((Odom_Data::deltaYTrack/Odom_Data::deltaTheta)+YTrackDist);
    }


    if (Odom_Data::XLocal == 0 && Odom_Data::YLocal == 0){
        Odom_Data::local_polar_angle = 0;
        Odom_Data::local_polar_position = 0;
    } else {
        Odom_Data::local_polar_angle = atan2(Odom_Data::YLocal, Odom_Data::XLocal); 
        Odom_Data::local_polar_position = sqrt(pow(Odom_Data::XLocal, 2) + pow(Odom_Data::YLocal, 2)); 
    }

    Odom_Data::global_polar_angle = Odom_Data::local_polar_angle - Odom_Data::previousTheta - (Odom_Data::deltaTheta/2);


    Odom_Data::xPosGlobal += Odom_Data::local_polar_position*cos(Odom_Data::global_polar_angle);
    Odom_Data::yPosGlobal += Odom_Data::local_polar_position*sin(Odom_Data::global_polar_angle);

    //loop every 5 milliseconds
    task::sleep(5); // was 10

  }
  return 1;
}

double targetX = 0; // Target X to drive to
double targetY = 0; // Target Y to drive to
double targetAngle = 0; // Target angle to face

// Controls if the feedback loop runs or not
bool runControl = false;

// After the timeout value is reached, the robot won't move
int timeOutValue = 2500;


/*
NOTES

Bottom left corner of field is (0,0)

*/

void driveToPoint(double Target_X, double Target_Y, double timeOutLength) {
  targetX = Target_X-0.890625; // Set target X
  targetY = Target_Y-0.890625; // Set target Y

  targetAngle = radToDeg(atan2(Target_Y - Odom_Data::yPosGlobal, Target_X - Odom_Data::xPosGlobal));

  timeOutValue = timeOutLength; // Set timeout
  runControl = true; // Let the feedback loop move the drivetrain
  Drivetrain_Timeout_timer.resetTimer(); // Reset and start countdown
}


void turnToAngle(double Target_Angle, double timeOutLength) {
  targetAngle = Target_Angle;

  timeOutValue = timeOutLength; // Set timeout

  // Set target X and Y as current position
  targetX = Odom_Data::xPosGlobal;
  targetY = Odom_Data::yPosGlobal;
  runControl = true; // Let the feedback loop move the drivetrain
  Drivetrain_Timeout_timer.resetTimer(); // Reset and start countdown
}

void turnToPoint(double xCoordToFace, double yCoordToFace, double timeOutLength) {
  
  // calculate angle
  targetAngle = radToDeg(atan2(yCoordToFace - Odom_Data::yPosGlobal, xCoordToFace - Odom_Data::xPosGlobal));

  // Set X and Y as current position
  targetX = Odom_Data::xPosGlobal;
  targetY = Odom_Data::yPosGlobal;
  //toggleClamp = true;
  timeOutValue = timeOutLength; // Set timeout

  runControl = true; // Let the feedback loop move the drivetrain
  Drivetrain_Timeout_timer.resetTimer(); // Reset and start countdown
}

//! PID

double PID_Data::error = 0; // Target distance difference
double PID_Data::prevError = 0; // Last cycle distance difference

double PID_Data::integral = 0; // The accumulated error
const double startIntegral = 15; // The minium distance to the target to use the integral
    // 1.5

double PID_Data::derivative = 0; // The change in error between cycles (prev and current)

const double kP = 0.95; // Error Gain 
    // 0.95 // 0.975-0.99 too high
const double kI = 0.0; // Integral Gain 
    //0.0045 // 0.008// 0.007 //0.006 // 0.005
const double kD = 0.0; // Derivative Gain
    //0.255 // 0.255 // 0.40 // 0.20 //0.25

double PID_Data::drivePowerPID = 0; // The power that will be sent to the motors


double PID_Data::turn_Error = 0; // Target angle difference
double PID_Data::turn_PrevError = 0; // Last cycle angle difference

double PID_Data::turn_Integral = 0; // The accumulated error
const double start_Turn_Integral = 0;
    // 0.17453293; // 10 degrees // 0.34906585; // Minimum angle to target to use the integral (20 degrees)

double PID_Data::turn_Derivative = 0; // The change in error between cycles (prev and current)

const double turn_kP = 0; // Turn Error Gain 
    // 0.1;//4.75; // 1.50 // 1.25 // 1.125 // 1.0625 // 1 // 2
const double turn_kI = 0; // Turn Integral Gain 
    //0.45; // 0.3 //0.1
const double turn_kD = 0; // Turn Derivative Gain
    //1.0; // 0.45; //0.5

double PID_Data::turnPowerPID = 0; // The power that will be sent to the motors

void PID(){

    // Calculate the distance from the current point to the target point
    PID_Data::error = sqrt(pow(targetX - Odom_Data::xPosGlobal, 2.0) + pow(targetY - Odom_Data::yPosGlobal, 2.0));

    // If the initial displacement is negative, negate the error
    if (((targetX - Odom_Data::xPosGlobal) + (targetY - Odom_Data::yPosGlobal)) < 0) PID_Data::error *= -1;
    

    PID_Data::turn_Error = reduce_negative_180_to_180(targetAngle - reduce_0_to_360( Inert.rotation()/* *360.0/gyro_scale*/ ));

    //only use integral if close enough to target but allows for tolerance
    if (fabs(PID_Data::error) < startIntegral){
        PID_Data::integral+=PID_Data::error;
    }

    if (fabs(PID_Data::turn_Error) < start_Turn_Integral){
        PID_Data::integral+=PID_Data::error;
    }

    // Checks if the error has crossed 0, and if it has, it eliminates the integral term.
    if ((PID_Data::error>0 && PID_Data::prevError<0)||(PID_Data::error<0 && PID_Data::prevError>0)){ 
        PID_Data::integral = 0; 
    }

    // Checks if the error has crossed 0, and if it has, it eliminates the integral term.
    if ((PID_Data::turn_Error>0 && PID_Data::turn_PrevError<0)||(PID_Data::turn_Error<0 && PID_Data::turn_PrevError>0)){ 
        PID_Data::integral = 0; 
    }

    // calculate the derivative
    PID_Data::derivative = PID_Data::error - PID_Data::prevError;
    PID_Data::turn_Derivative = PID_Data::turn_Error - PID_Data::turn_PrevError;


    // calculate the power output
    PID_Data::drivePowerPID = PID_Data::error * kP + PID_Data::integral * kI + PID_Data::derivative * kD;
    PID_Data::turnPowerPID = (PID_Data::turn_Error * turn_kP + 
                              PID_Data::turn_Integral * turn_kI + 
                              PID_Data::turn_Derivative * turn_kD);
    
    // If in tolerance range, cut power
    /*if(fabs(PID_Data::error) < maxError) PID_Data::drivePowerPID = 0;
    if(fabs(PID_Data::turn_Error) < turn_MaxError) PID_Data::turnPowerPID = 0;*/

    // Update previous for next loop
    PID_Data::prevError = PID_Data::error;
    PID_Data::turn_PrevError = PID_Data::turn_Error;
}


int driveControl() {

  //loop to constantly execute commands and update values
  while(1) {
    
    //Check if we should run the control
    if(runControl) {

      //get PID values for driving and turning
      PID();

      //set power for each motor
      leftFront.spin(fwd, PID_Data::drivePowerPID + PID_Data::turnPowerPID, pct);
      leftMid.spin(fwd, PID_Data::drivePowerPID + PID_Data::turnPowerPID, pct);
      leftBack.spin(fwd, PID_Data::drivePowerPID + PID_Data::turnPowerPID, pct);
      rightBack.spin(fwd, PID_Data::drivePowerPID - PID_Data::turnPowerPID, pct);
      rightMid.spin(fwd, PID_Data::drivePowerPID - PID_Data::turnPowerPID, pct);
      rightFront.spin(fwd, PID_Data::drivePowerPID - PID_Data::turnPowerPID, pct);

      // If the time out is reached, stop the loop
      if(Drivetrain_Timeout_timer.timer(timeUnits::msec) > timeOutValue) {
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
    task::sleep(5);
  }
  return 1;
}


void calibrate_IMU(){
  Inert.calibrate();

  // indicate calibrating on the controller screen
  while (Inert.isCalibrating()){
    task::sleep(100);
  } 

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
  leftVolt = clamp(12 * (ControlSense(forward + turn, 2) / 100.0), -12.0, 12.0);
  rightVolt = clamp(12 * (ControlSense(forward - turn, 2) / 100.0), -12.0, 12.0);

  // Apply leftVolt to the left motors
  leftFront.spin(fwd, leftVolt, volt);
  leftMid.spin(fwd, leftVolt, volt);
  leftBack.spin(fwd, leftVolt, volt);
  // Apply rightVolt to the right motors
  rightBack.spin(fwd, rightVolt, volt);
  rightMid.spin(fwd, rightVolt, volt);
  rightFront.spin(fwd, rightVolt, volt);

}

bool isIntakeStuck(){
  return std::abs(intake.value()) == 0;
}

std::vector<double> get_Motor_Data(){

  // create a temp vector to store the data
  std::vector<double> data(5, 0.0); // Initialize data vector with zeros

  /* index 0 = temp (C)
     index 1 = Velocity (DEG/SEC)
     index 2 = RPM
     index 3 = Voltage
     index 4 = Efficiency */

  // Get data from each motor
  for(uint8_t i = 0; i < 6; i++){
    data[0] += MotorList[i].temperature(celsius);
    data[1] += MotorList[i].velocity(velocityUnits::dps);
    data[2] += MotorList[i].convertVelocity(data[1], dps, rpm);
    data[3] += MotorList[i].voltage(volt);
    data[4] += MotorList[i].efficiency();
  }
  
  // Calculate the average
  for (auto &dat : data) dat /= 6.0;
  
  return data; // Return the vector containing average values
}

std::vector<double> get_Auton_Data(PID_Data &PDat, Odom_Data &ODat){
  return {

    /*---PID Data---*/

    // Linear PID
    PDat.error, PDat.prevError, PDat.integral, PDat.derivative, PDat.drivePowerPID,

    // Turning PID
    PDat.turn_Error, PDat.turn_PrevError, PDat.turn_Integral, PDat.turn_Derivative, PDat.turnPowerPID,
  
    /*---Odom Data---*/
  
    // Parallel sensor data
    ODat.currentY, ODat.previousY, ODat.deltaYTrack,
    // Perpendicular sensor data
    ODat.currentX, ODat.previousX, ODat.deltaXTrack,
    // Inertial sensor data
    radToDeg(ODat.currentTheta), radToDeg(ODat.previousTheta), radToDeg(ODat.deltaTheta),
    // Robot-relative X and Y
    ODat.XLocal, ODat.YLocal,
    // Polarized angle and position of the robot-relative X and Y
    ODat.local_polar_angle, ODat.local_polar_position,
    // Global positions
    radToDeg(ODat.global_polar_angle), ODat.xPosGlobal, ODat.yPosGlobal
  };
}