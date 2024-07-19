#pragma once
#ifndef ODOMETRY_HPP_
#define ODOMETRY_HPP_

#include "position.hpp"

/** 
* @brief This class holds the odometry data
*/
class Odom_Data{
    public:
        Odom_Data();
        ~Odom_Data();

        static double LPos; // The current position of the left wheel (DEGREES)
        static double RPos; // The current position of the right wheel (DEGREES)
        static double SPos; // The current position of the side wheel (DEGREES)
        
        static double currentAbsoluteOrientation; //The current angle of the bot (RADIANS)

        static double LPrevPos; // The previous position of the left wheel (DEGREES)
        static double RPrevPos; // The previous position of the right wheel (DEGREES)
        static double SPrevPos; // The previous position of the side wheel (DEGREES)
        static double prevTheta; // The previous angle of the bot (RADIANS)

        static double deltaDistL; // The distance traveled by the left wheel (INCHES)
        static double deltaDistR; // The distance traveled by the right wheel (INCHES)
        static double deltaDistS; // The distance traveled by the side wheel (INCHES)
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