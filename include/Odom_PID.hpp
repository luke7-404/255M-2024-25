#pragma once
#ifndef ODOM_PID_HPP_
#define ODOM_PID_HPP_

#include "position.hpp"
#include "cmath"

extern double X_START;
extern double Y_START;
 

/** 
* @brief This class holds the odometry data
*/
class Odom_Data{
    public:
        static double currentY; // The current position of the parallel wheel (DEGREES)
        static double previousY; // The previous position of the parallel wheel (DEGREES)
        static double deltaYTrack; // The distance traveled by the parallel wheel (INCHES)

        static double currentX; // The current position of the perpendicular wheel (DEGREES)
        static double previousX; // The previous position of the perpendicular wheel (DEGREES)
        static double deltaXTrack; // The distance traveled by the perpendicular wheel (INCHES)

        static double currentTheta; //The current angle of the bot (RADIANS)
        static double previousTheta; // The previous angle of the bot (RADIANS)
        static double deltaTheta; // The change in angle of the bot (RADIANS)

        static double XLocal; // The change in X position relative to the robot (INCHES)
        static double YLocal; // The change in Y position relative to the robot (INCHES)

        static double local_polar_angle; // The change in the polar angle of the robot (RADIANS)
        static double local_polar_position; // The change in the polar position of the robot (INCHES)

        static double global_polar_angle; // The global angle of the bot relative to field (RADIANS)
        
        static double xPosGlobal; // Global X position of the bot relative to the field (INCHES)
        static double yPosGlobal; // Global Y position of the bot relative to the field (INCHES)
        
        /**
         * @brief Set the Position object
         * 
         * @param x // Desired X position to set to
         * @param y // Desired Y position to set to
         */
        void setPosition(double x, double y);
};    


/**
 * @brief This class holds the static variables for the PID calculations
 * 
 */
class PID_Data {
    public:

        /**
         * @brief This variable holds the error value for the PID calculation
         */
        static double error;
        /**
         * @brief This variable holds the previous error value for the PID calculation
         */
        static double prevError;
        /**
         * @brief This variable holds the integral value for the PID calculation
         */
        static double integral;
        /**
         * @brief This variable holds the derivative value for the PID calculation
         */
        static double derivative;
        /**
         * @brief This variable holds the error value for the turn PID calculation
         */
        static double turn_Error;
        /**
         * @brief This variable holds the previous error value for the turn PID calculation
         */
        static double turn_PrevError;
        /**
         * @brief This variable holds the integral value for the turn PID calculation
         */
        static double turn_Integral;
        /**
         * @brief This variable holds the derivative value for the turn PID calculation
         */
        static double turn_Derivative;
        /**
         * @brief This variable holds the PID power output for the drive
         */
        static double drivePowerPID;
        /**
         * @brief This variable holds the PID power output for the turn
         */
        static double turnPowerPID;
};


#endif