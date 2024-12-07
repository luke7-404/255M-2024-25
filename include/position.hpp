#pragma once
#ifndef POSITION_HPP_
#define POSITION_HPP_

#include "vex.h"


/**
 * @brief Prints the current position data of the robot on the screen.
 * 
 * Calculates and prints how far the X and Y values are from 0 (or where the robot needs to go).
 * Tells whether the robot is using the right sensor and shows the bot's displacement based on 
 * the selected autonomous program
 * 
 * @param autoNum The selected autonomous program
 */
extern void positionRobot(short autoNum);

/**
 * @brief Gets the calculated X displacement from the middle of the robot
 * 
 * @note To use properly, make sure the isRight boolean value is correctly assigned
 * 
 * @return Returns X-axis displacement (double) 
 */
extern double getXDisplacement();

/**
 * @brief Gets the calculated X displacement from the middle of the robot
 * 
 * @return Returns Y-axis displacement (double)
 */
extern double getYDisplacement();


#endif