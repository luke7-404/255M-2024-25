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
 * @param reverse sets the reverse override 
 * @param timeOutLength The timeout length in milliseconds (default is 2500)
 */
extern void driveToPoint(double Target_X, double Target_Y, double Angle_target, uint8_t reverse = 2, double timeOutLength = 2500.0);

/**
 * @brief Turns the robot to the specified angle with a specified timeout
 * @param targetAngle The target angle to face
 * @param timeOutLength The timeout length in milliseconds (default is 1500)
 */
extern void turnToAngle(double targetAngle, double timeOutLength = 1500.0);

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
 * @brief Updates the PID calculation for driving
 */
double ControlSense(int percent);

/**
 * @brief Controls the drivetrain using joystick inputs.
 * 
 * This function takes two parameters representing the forward and turning
 * inputs from a joystick. It processes these inputs to control the movement
 * of the drivetrain, allowing for both linear and rotational motion.
 * 
 * @param forward The forward/backward input value from the joystick. 
 *                Positive values indicate forward motion, while negative 
 *                values indicate backward motion.
 * @param turn The left/right turning input value from the joystick. 
 *             Positive values indicate a right turn, while negative 
 *             values indicate a left turn.
 */
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
 * @brief Resets the state of the LadyBrown mechanism.
 * 
 * This function is used to reset any internal states or configurations
 * of the LadyBrown mechanism to their default values.
 */
extern void reset_LadyBrown();

/**
 * @brief Moves the LadyBrown mechanism to a specified angle.
 * 
 * This function controls the movement of the LadyBrown mechanism to
 * reach a specified target angle. It can optionally reset the sensor
 * readings before starting the movement.
 * 
 * @param targetAngle The desired angle to move the LadyBrown mechanism to.
 * @param resetSense A boolean flag indicating whether to reset sensor
 *                   readings before moving. `true` to reset, `false` otherwise.
 */
extern void moveLadyBrown(double targetAngle, bool resetSense);

// Represents the current stage or phase of the LadyBrown operation.
extern uint8_t stage;

/**
 * @brief Controls the LadyBrown mechanism.
 * 
 * This function manages the overall control logic for the LadyBrown
 * mechanism, coordinating its operations and returning a status code.
 * 
 * @return int A status code indicating the result of the control operation.
 */
extern int ladyBrownControl();

/**
 * @brief Executes the LadyBrown routine.
 * 
 * This function runs the predefined routine for the LadyBrown mechanism,
 * executing a series of operations in sequence.
 */
extern void LadyBrownRoutine();

#endif 