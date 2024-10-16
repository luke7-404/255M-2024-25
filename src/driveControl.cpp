#include "odometry.hpp"
#include "drivetrain.hpp"

//* CONSTANTS */
//Radius of tracking wheels in inches
const double WHEEL_RADIUS = 1.375;

//* Starting angle relative to field) (RADIANS)
const double THETA_START = M_PI_2; //imagine the field is a unit circle

//The starting x and y coordinates of the bot (INCHES)
  //* These distances are relative to bottom left corner (0,0) on the field
const double X_START = 72; //x_distance;
const double Y_START = 48; //sensorY;

//Distances of tracking wheels from robot's center (INCHES)
const double YTrackDist = 2.250;
const double XTrackDist = 2.250;

//* Calculated Values (every loop)
//Angles (DEGREES) 
  //! Is converted to radians for math
double Odom_Data::YTrackPos = 0;
double Odom_Data::XTrackPos = 0;

double Odom_Data::YPrevPos = 0;
double Odom_Data::XPrevPos = 0;

//Distances traveled by tracking wheels each loop (INCHES)
double Odom_Data::deltaDistY = 0;
double Odom_Data::deltaDistX = 0;

//The current angle of the bot (RADIANS)
double Odom_Data::currentAbsoluteOrientation = THETA_START;

//The previous angle of the bot (RADIANS)
double Odom_Data::prevTheta = THETA_START;

//The change in Theta each loop (RADIANS)
double Odom_Data::deltaTheta = 0;

//The Average angle Theta (In RADIANS) throughout the arc
  //currentAbsoluteOrientation + (deltaTheta / 2)
double Odom_Data::avgThetaForArc = currentAbsoluteOrientation + (Odom_Data::deltaTheta / 2);

//The changes in the robot's X and Y positions (INCHES)
double Odom_Data::deltaXLocal = 0;
double Odom_Data::deltaYLocal = 0;

//The change in the X and Y offsets converted from the robot's position (INCHES)
double Odom_Data::deltaXGlobal = 0;
double Odom_Data::deltaYGlobal = 0;

//The global position of the bot relative to field (INCHES)
double Odom_Data::xPosGlobal = X_START;
double Odom_Data::yPosGlobal = Y_START;

int positionTracking() {
  while(1) {
    //Get encoder values (DEGREES)
    
    //Update previous values to be used next loop (DEGREES)
    Odom_Data::YPrevPos = Odom_Data::YTrackPos;
    Odom_Data::XPrevPos = Odom_Data::XTrackPos;

    //Update the previous Theta value (RADIANS)  
    Odom_Data::prevTheta = Odom_Data::currentAbsoluteOrientation;

    Odom_Data::YTrackPos = YTrack.position(rotationUnits::deg);
    Odom_Data::XTrackPos = XTrack.position(rotationUnits::deg);

    //Calculate distance traveled by tracking each wheel (INCHES)
      //Converts degrees to radians
    Odom_Data::deltaDistY = ((Odom_Data::YTrackPos - Odom_Data::YPrevPos) * M_PI / 180) * WHEEL_RADIUS;
    Odom_Data::deltaDistX = ((Odom_Data::XTrackPos - Odom_Data::XPrevPos) * M_PI / 180) * WHEEL_RADIUS;

    //Calculate the current absolute orientation (RADIANS)
    Odom_Data::currentAbsoluteOrientation = std::fmod((360 - Inert.heading(rotationUnits::deg) + 180), 360) * M_PI / 180.0; // (360 + Inert.heading(rotationUnits::deg)) * M_PI / 180.0;

    //Calculate the change in the angle of the bot (RADIANS)
    Odom_Data::deltaTheta = Odom_Data::currentAbsoluteOrientation - Odom_Data::prevTheta;

    //If we didn't turn, then we only translated
    if(Odom_Data::deltaTheta == 0) {
      Odom_Data::deltaXLocal = Odom_Data::deltaDistX;
      // could be either L or R, since if deltaTheta == 0 we assume they're =
      Odom_Data::deltaYLocal = Odom_Data::deltaDistY;
    }
    //Else, calculate the new local position
    else {
      //Calculate the changes in the X and Y values (INCHES)
      //General equation is:
        //Distance = 2 * Radius * sin(deltaTheta / 2)
      Odom_Data::deltaXLocal = 2 * sin(Odom_Data::deltaTheta / 2.0) * ((Odom_Data::deltaDistX / Odom_Data::deltaTheta) + XTrackDist);
      Odom_Data::deltaYLocal = 2 * sin(Odom_Data::deltaTheta / 2.0) * ((Odom_Data::deltaDistY / Odom_Data::deltaTheta) - YTrackDist);

    }

    //The average angle of the robot during it's arc (RADIANS)
    Odom_Data::avgThetaForArc = Odom_Data::currentAbsoluteOrientation - (Odom_Data::deltaTheta / 2);

    Odom_Data::deltaXGlobal = (Odom_Data::deltaYLocal * cos(Odom_Data::avgThetaForArc)) - (Odom_Data::deltaXLocal * sin(Odom_Data::avgThetaForArc));
    Odom_Data::deltaYGlobal = (Odom_Data::deltaYLocal * sin(Odom_Data::avgThetaForArc)) + (Odom_Data::deltaXLocal * cos(Odom_Data::avgThetaForArc));

    //Wraps angles back around if they ever go under 0 or over 2 pi
    while(Odom_Data::currentAbsoluteOrientation >= 2 * M_PI) {
      Odom_Data::currentAbsoluteOrientation -= 2 * M_PI;
    }
    
    while(Odom_Data::currentAbsoluteOrientation < 0) {
      Odom_Data::currentAbsoluteOrientation += 2 * M_PI;
    }

    //Update global positions
    Odom_Data::xPosGlobal += Odom_Data::deltaXGlobal;
    Odom_Data::yPosGlobal += Odom_Data::deltaYGlobal;


    //loop every 20 milliseconds
    task::sleep(10); // was 10

  }
  return 1;
}

