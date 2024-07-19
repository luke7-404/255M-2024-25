#pragma once
#ifndef DRIVETRAIN_HPP_
#define DRIVETRAIN_HPP_

#include "odometry.hpp"

/**
 * @brief This class holds the static variables for the PID calculations
 * 
 */
class PID_Data {
    public:
        /**
         * @brief Constructor for the PID_Data object
         */
        PID_Data();
        /**
         * @brief Destructor for the PID_Data object
         */
        ~PID_Data();
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

/**
 * @brief This variable holds the target X position to drive to
 */
extern double targetX;

/**
 * @brief This variable holds the target Y position to drive to
 */
extern double targetY;

/**
 * @brief This variable holds the target angle to face while driving
 */
extern double targetAngle;

/**
 * @brief This variable controls if the feedback loop is running
 */
extern bool runControl;

/**
 * @brief Drives the robot to the specified coordinates with a specified timeout
 * @param xTarget The target X position to drive to
 * @param yTarget The target Y position to drive to
 * @param targetAngle The target angle to face while driving
 * @param timeOutLength The timeout length in milliseconds (default is 2500)
 */
extern void driveTo(double xTarget, double yTarget, double targetAngle, double timeOutLength = 2500);

/**
 * @brief Turns the robot to the specified angle with a specified timeout
 * @param targetAngle The target angle to face
 * @param timeOutLength The timeout length in milliseconds (default is 2500)
 */
extern void turnTo(double targetAngle, double timeOutLength = 2500);

/**
 * @brief Turns the robot to the specified coordinate with a specified timeout
 * @param xCoordToFace The X coordinate to face
 * @param yCoordToFace The Y coordinate to face
 * @param timeOutLength The timeout length in milliseconds (default is 1500)
 */
extern void turnToPoint(double xCoordToFace, double yCoordToFace, double timeOutLength = 1500);

/**
 * @brief Calibrates the IMU
 */
extern void calibrate_IMU();

/**
 * @brief Resets the tracking wheels
 */
extern void reset_Tracking_Wheels();


/**
 * @brief Gets the average temperature, voltage, current, torque, and efficiency of the drivetrain motors
 * @return A vector containing the averages of the motors
 */
extern std::vector<double> get_Motor_Data();

/**
 * @brief Converts radians to degrees
 * @param rad The angle in radians
 * @return The angle in degrees
 */
extern double radToDeg(double rad);

/**
 * @brief Updates the PID calculation for driving
 */
void drivePID();

/**
 * @brief Updates the PID calculation for turning
 */
void turnPID();

/**
 * @brief Controls the drive motor power based on the PID calculation
 * @return The control value for the drive motors
 */
int driveControl();

#endif 