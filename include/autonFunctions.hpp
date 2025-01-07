#pragma once
#ifndef AUTONFUNCTIONS_HPP
#define AUTONFUNCTIONS_HPP

#include "main.h"
#include "lemlib/api.hpp"

using namespace pros;

// The distance sensor that detects the claw
Distance clawDistance(17);

/**
 * @class Auton_Functions
 * 
 * @brief This class provides autonomous functions for a robot, including control over
 *        various pneumatic components and motor groups. It supports different autonomous
 *        routines for both red and blue sides of the field, as well as skills challenges.
 * 
 * @details The class manages the state of the robot's claw using an enum `clawState` and
 *          provides methods to set the claw state and perform automatic clamping. It also
 *          includes nested structures for red and blue autonomous routines, each with
 *          specific strategies like goal rush and ring rush.
 * 
 */
class Auton_Functions{

    public:
        /**
         * @brief Constructs an Auton_Functions object with specified components.
         * 
         * 
         * @param drive Reference to the robot's chassis for movement control.
         * @param intakeRaiser Reference to the pneumatic component controlling the intake raiser.
         * @param mogoClaw Reference to the pneumatic component controlling the mobile goal claw.
         * @param doinker Reference to the pneumatic component controlling the doinker.
         * @param intake Reference to the motor group managing the intake mechanism.
         */
        Auton_Functions(lemlib::Chassis& drive, ADIPneumatics& intakeRaiser, 
                        ADIPneumatics& mogoClaw, ADIPneumatics& doinker, MotorGroup& intake)
        // Assign parameter values to member variables
        : drive(drive), intakeRaiser(intakeRaiser), mogoClaw(mogoClaw), doinker(doinker), intake(intake) {}

        /**
         * @brief 
         * Automatically controls the clamp claw mechanism.
         * This function is designed to engage or release the clamp claw
         * based on predefined autonomous routines.
         */
        void autoClampClaw();

        /**
         * @enum clawState
         * @brief Represents the possible states of a robotic claw.
         * 
         * This enumeration defines three states for controlling a robotic claw:
         * - OPEN: represented by the value 1.
         * - CLOSE: represented by the value -1.
         * - AUTO: represented by the value 0.
         */
        enum clawState {
            OPEN = 1, // The claw is open
            CLOSE = -1, // The claw is closed
            AUTO = 0 // The claw operates in automatic mode (based on sensor readings)
        };

        /**
         * @brief Sets the state of the claw mechanism.
         *
         * @param value The desired state to set the claw to, represented by the clawState enum.
         */
        void setClawState(clawState value);

        void AWP1(); // Left Side of field
        void AWP2(); // Right Side of field
        void Skills(); // Skills is skills

        /**
         * @brief Represents the autonomous routine for the RED alliance.
         *
         * This structure is used to define and manage the actions and
         * strategies specific to the RED alliance
         * during the autonomous phase.
         */
        struct RED_Auton {
                // Assign parameter value to member variable
                RED_Auton(Auton_Functions& parent) : parent(parent){}
                ~RED_Auton(){} // Destructor so that C++ can delete object

                /**
                 * @brief Executes the autonomous routine for the middle goal rush on Red side
                 */
                void goalRush();

                /**
                 * @brief Executes the autonomous routine for middle rings rush on Red side
                 */
                void ringRush();
            private:
                /**
                 * @brief A reference to the parent Auton_Functions object.
                 * 
                 * This member variable is used to access the functionalities
                 * provided by the parent Auton_Functions instance.
                 */
                Auton_Functions& parent;
        };

        /**
         * @brief Represents the autonomous routine for the BLUE alliance.
         *
         * This structure is used to define and manage the actions and
         * strategies specific to the BLUE alliance during the 
         * autonomous phase
         */
        struct BLUE_Auton {
                // Assign parameter value to member variable
                BLUE_Auton(Auton_Functions& parent) : parent(parent){}
                ~BLUE_Auton(){} // Destructor so that C++ can delete object

                /**
                 * @brief Executes the autonomous routine for the middle goal rush on Blue side
                 */
                void goalRush();

                /**
                 * @brief Executes the autonomous routine for middle rings rush on Blue side
                 */
                void ringRush();
            private:
                /**
                 * @brief A reference to the parent Auton_Functions object.
                 * 
                 * This member variable is used to access the functionalities
                 * provided by the parent Auton_Functions instance.
                 */
                Auton_Functions& parent;
        };
    
    /**
     * @brief Private member variables for controlling various robot components.
     */
    private:
        lemlib::Chassis& drive; // Reference to the lemlib Chassis object for driving control 
        ADIPneumatics& intakeRaiser; // Reference to the ADIPneumatics object for controlling the intake raiser 
        ADIPneumatics& mogoClaw; // Reference to the ADIPneumatics object for controlling the mobile goal claw
        ADIPneumatics& doinker; // Reference to the ADIPneumatics object for controlling the doinker mechanism 
        MotorGroup& intake; // Reference to the MotorGroup object for controlling the intake motors
        static clawState release; // Static variable representing the state of the claw release mechanism
};


/**
 ** Defines various ABSOLUTE coordinate points and poses for a robotics field.
 * 
 * These macros represent specific locations on the field, including goals,
 * rings, and corners, for both the bottom left and bottom right sections.
 * Additional points are defined for middle and top sections, as well as
 * specific positions for red and blue wall stakes.
 * 
 * Each macro is defined with coordinates (x, y) and, where applicable, 
 * an orientation angle.
 * 
 * ! PERSPECTIVE FROM RED DRIVER BOX
 * ! (0,0) IS AT BOTTOM LEFT CORNER
 */

#define bottomLeftGoal_POSE 48,24,180 
#define bottomLeftRingA_POINT 48,48
#define bottomLeftRingB_POINT 24,48
#define bottomLeftRingC_POINT 24,24
#define bottomLeftRingD_POINT 24,12
#define bottomLeftRingE_POINT 12,24
#define bottomLeftCorner_POSE 0,0,225

#define bottomRightGoal_POSE 96,24,90
#define bottomRightRingA_POINT 96,48
#define bottomRightRingB_POINT 120,48
#define bottomRightRingC_POINT 120,24
#define bottomRightRingD_POINT 120,12
#define bottomRightRingE_POINT 132,24
#define bottomRightCorner_POSE 144,0,135

#define middleLeftRing_POINT 12,72
#define middleCenterRing_POINT 72,72
#define middleCenterRing_POINT 132,72

#define topMiddleGoal_POINT 72,120
#define topRingA_POINT 120,96
#define topRingB_POINT 48,96
#define topRingC_POINT 24,96
#define topRingD_POINT 24,120
#define topRingE_POINT 120,120

#define redWallStake_POSE 72,6,0
#define blueWallStake_POSE 72,138,180
#define blueWallStakeRing_POINT 96,96


#endif