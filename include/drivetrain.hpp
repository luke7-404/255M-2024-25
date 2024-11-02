#pragma once
#ifndef DRIVETRAIN_HPP_
#define DRIVETRAIN_HPP_

#include "vex.h"


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
extern void driveToPoint(double xTarget, double yTarget, double timeOutLength = 2500);

/**
 * @brief Turns the robot to the specified angle with a specified timeout
 * @param targetAngle The target angle to face
 * @param timeOutLength The timeout length in milliseconds (default is 2500)
 */
extern void turnToAngle(double targetAngle, double timeOutLength = 2500);

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
 * @brief Gets the data from the PID and Odometry processes in that order
 * 
 * @param PDat A PID object for data collection
 * @param ODat An Odom object for data collection
 * 
 * @return A vector containing the data from PID and Odom
 */
extern std::vector<double> get_Auton_Data(PID_Data &PDat, Odom_Data &ODat);

/**
 * @brief Converts radians to degrees
 * @param rad The angle in radians
 * @return The angle in degrees
 */
extern double radToDeg(double rad);

/**
 * @brief Updates the PID calculation for driving
 */
double ControlSense(double percent, int selectedCurve = 0);

extern void joeySticks(int32_t forward, int32_t turn);

/**
 * @brief Updates the PID calculation for turning and driving
 */
void PID();

/**
 * @brief Controls the drive motor power based on the PID calculation
 * @return The control value for the drive motors
 */
extern int driveControl();

/**
 * @brief Calculates the X and Y position of the robot from its center
 * @return The X, Y, and current heading
 */
extern int positionTracking();


/**
 * @brief Checks if the intake motor's torx is greater than 1
 * 
 * @return true 
 * @return false 
 */
extern bool isIntakeStuck();

#endif 