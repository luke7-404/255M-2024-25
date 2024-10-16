#pragma once
#ifndef ODOMETRY_HPP_
#define ODOMETRY_HPP_

#include "position.hpp"
#include "cmath"

/** 
* @brief This class holds the odometry data
*/
class Odom_Data{
    public:
        Odom_Data();
        ~Odom_Data();

        static double YTrackPos; // The current position of the parallel wheel (DEGREES)
        static double XTrackPos; // The current position of the perpendicular wheel (DEGREES)
        
        static double currentAbsoluteOrientation; //The current angle of the bot (RADIANS)

        static double YPrevPos; // The previous position of the parallel wheel (DEGREES)
        static double XPrevPos; // The previous position of the perpendicular wheel (DEGREES)
        static double prevTheta; // The previous angle of the bot (RADIANS)

        static double deltaDistY; // The distance traveled by the parallel wheel (INCHES)
        static double deltaDistX; // The distance traveled by the perpendicular wheel (INCHES)
        static double deltaTheta; // The change in angle of the bot (RADIANS)

        static double avgThetaForArc; // The average angle of the bot (RADIANS)

        static double deltaXLocal; // Change in X position relative to robot (INCHES)
        static double deltaYLocal; // Change in Y position relative to robot (INCHES)

        static double deltaXGlobal; // Change in X position relative to the field (INCHES)
        static double deltaYGlobal; // Change in Y position relative to the field (INCHES)
        
        static double xPosGlobal; // Global X position of the bot relative to the field (INCHES)
        static double yPosGlobal; // Global Y position of the bot relative to the field (INCHES)

};

//The odometry function
extern int positionTracking();



#endif