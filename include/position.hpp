#pragma once
#ifndef POSITION_HPP_
#define POSITION_HPP_

#include "main.h" // add pros header


/**
 * @brief Used to determine if data is read from the right side.
 */
extern bool isRight;

/**
 * @brief Prints the current position data of the robot on the screen.
 * 
 * Calculates and prints how far the X and Y values are from 0 (or where the robot needs to go).
 * Tells whether the robot is using the right sensor and shows the bot's displacement based on 
 * the selected autonomous program
 * 
 * @param autonID The character that represents the selected autonomous program
 */
extern void positionRobot(char autonID);

/**
 * @brief Gets the calculated X displacement from the middle of the robot
 * 
 * @note To use properly, make sure the isRight boolean value is correctly assigned
 * 
 * @return Returns X-axis displacement in inches (float) 
 */
extern float getXDisplacement();

/**
 * @brief Gets the calculated X displacement from the middle of the robot
 * 
 * @return Returns Y-axis displacement in inches (float)
 */
extern float getYDisplacement();

/**
 * @brief Converts a length from centimeters to inches.
 *
 * @param cm The length in centimeters to be converted.
 * 
 * @return The equivalent length in inches.
 */
extern float cmToInches(float cm);


#endif