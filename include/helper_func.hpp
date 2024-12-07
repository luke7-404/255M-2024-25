#pragma once
#ifndef HELPER_FUNC_HPP
#define HELPER_FUNC_HPP

#include "math.h"

/**
 * "Clamps" a number between a min and max.
 * Does no error checking and assumes min is less
 * than or equal to max.
 * 
 * @param input The number to be clamped.
 * @param min Minimum edge of the clamp.
 * @param max Maximum edge of the clamp.
 * @return Clamped number.
 */
extern double clamp(double input, double min, double max);

/**
 * Converts an angle in radians to degrees.
 * 
 * @param angle The angle in radians.
 * @return Angle in degrees.
 */
extern double radToDeg(double rad);

/**
 * Converts an angle in degrees to radians.
 * 
 * @param angle The angle in degrees.
 * @return Angle in radians.
 */
extern double degToRad(double deg);


/**
 * Scales a joystick to drive voltage scale.
 * Values get multiplied by 12 because motors can
 * output a max of 12 volts.
 * 
 * @param percent The input joystick reading.
 * @return The equivalent value in volts.
 */

float to_volt(float percent);

/**
 * Converts an angle to an equivalent one in the range [0, 360).
 * 
 * @param angle The angle to be reduced in degrees.
 * @return Reduced angle.
 */

float reduce_0_to_360(float angle);

/**
 * Converts an angle to an equivalent one in the range [-180, 180).
 * 
 * @param angle The angle to be reduced in degrees.
 * @return Reduced angle.
 */

float reduce_negative_180_to_180(float angle);

/**
 * @brief Get the Heading 
 * @param angle the inertial's rotation value 
 * @return double 
 */
double getHeading(double angle);

/**
 * @brief Convert inches to cm
 * @param inches inches
 * @return double 
 */
double inchesToCm(double inches);

/**
 * @brief Convert cm to inches
 * @param cm centimeters
 * @return double 
 */
double cmToInches(double cm);

#endif