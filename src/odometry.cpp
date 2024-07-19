#include "odometry.hpp"

Odom_Data::Odom_Data(){
  std::cout << "Odom_Data object created" << std::endl;
}

Odom_Data::~Odom_Data(){
  std::cout << "Odom_Data object destroyed" << std::endl;
}

//* CONSTANTS */
//Radius of tracking wheels in inches
const double WHEEL_RADIUS = 1.375;

//* Starting angle relative to field) (RADIANS)
const double THETA_START = M_PI; //imagine the field is a unit circle

//The starting x and y coordinates of the bot (INCHES)
  //* These distances are relative to bottom left corner (0,0) on the field
const double X_START = 72.0; //x_distance;
const double Y_START = 72.0; //sensorY;

//Distances of tracking wheels from robot's center (INCHES)
const double LTrackDist = 2.250;
const double RTrackDist = 2.250;
const double STrackDist = 5.8125;

//* Calculated Values (every loop)
//Angles (DEGREES) 
  //! Is converted to radians for math
double Odom_Data::LPos = 0;
double Odom_Data::RPos = 0;
double Odom_Data::SPos = 0;

double Odom_Data::LPrevPos = 0;
double Odom_Data::RPrevPos = 0;
double Odom_Data::SPrevPos = 0;

//Distances traveled by tracking wheels each loop (INCHES)
double Odom_Data::deltaDistL = 0;
double Odom_Data::deltaDistR = 0;
double Odom_Data::deltaDistS = 0;

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
    Odom_Data::LPrevPos = Odom_Data::LPos;
    Odom_Data::RPrevPos = Odom_Data::RPos;
    Odom_Data::SPrevPos = Odom_Data::SPos;

    Odom_Data::LPos = leftTrack.position(rotationUnits::deg);
    Odom_Data::RPos = rightTrack.position(rotationUnits::deg);
    Odom_Data::SPos = middleTrack.position(rotationUnits::deg);

    //Calculate distance traveled by tracking each wheel (INCHES)
      //Converts degrees to radians
    Odom_Data::deltaDistL = ((Odom_Data::LPos - Odom_Data::LPrevPos) * M_PI / 180) * WHEEL_RADIUS;
    Odom_Data::deltaDistR = ((Odom_Data::RPos - Odom_Data::RPrevPos) * M_PI / 180) * WHEEL_RADIUS;
    Odom_Data::deltaDistS = ((Odom_Data::SPos - Odom_Data::SPrevPos) * M_PI / 180) * WHEEL_RADIUS;

    //Calculate the current absolute orientation (RADIANS)
    Odom_Data::currentAbsoluteOrientation = (360 - Inert.heading(rotationUnits::deg)) * M_PI / 180.0;

    //Calculate the change in the angle of the bot (RADIANS)
    Odom_Data::deltaTheta = Odom_Data::currentAbsoluteOrientation - Odom_Data::prevTheta;

    //Update the previous Theta value (RADIANS)  
    Odom_Data::prevTheta = Odom_Data::currentAbsoluteOrientation;

    //If we didn't turn, then we only translated
    if(Odom_Data::deltaTheta == 0) {
      Odom_Data::deltaXLocal = Odom_Data::deltaDistS;
      // could be either L or R, since if deltaTheta == 0 we assume they're =
      Odom_Data::deltaYLocal = Odom_Data::deltaDistL;
    }
    //Else, calculate the new local position
    else {
      //Calculate the changes in the X and Y values (INCHES)
      //General equation is:
        //Distance = 2 * Radius * sin(deltaTheta / 2)
      Odom_Data::deltaXLocal = 2 * sin(Odom_Data::deltaTheta / 2.0) * ((Odom_Data::deltaDistS / Odom_Data::deltaTheta) + STrackDist);
      Odom_Data::deltaYLocal = 2 * sin(Odom_Data::deltaTheta / 2.0) * ((Odom_Data::deltaDistR / Odom_Data::deltaTheta) - RTrackDist);

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
    task::sleep(20); // was 10

  }
  return 1;
}
