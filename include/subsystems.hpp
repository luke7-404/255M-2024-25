#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// intake motor group - ports 14 (reversed), 13
inline pros::MotorGroup intake({20, -2});
inline pros::Motor firstStageIntake(20);
inline pros::Motor SecondStageIntake(-2);
inline pros::Motor ladyBrownMtr(-16);

//* PNEUMATICS
inline pros::adi::Pneumatics mogoClaw('H', false);
inline pros::adi::Pneumatics intakeRaiser('C', false);
inline pros::adi::Pneumatics rightDoinker('A', false);
inline pros::adi::Pneumatics leftDoinker('B', false);

// The distance sensor that detects the claw
inline pros::Distance clawDistance(15);

// The color sensor that detects the color of the ring
inline pros::Optical colorSensor(6);

//Ladybrown sensors
inline pros::Rotation ladyRot(7); // The ladybrown sensor on the left side