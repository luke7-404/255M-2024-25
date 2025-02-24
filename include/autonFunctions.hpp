#pragma once
#ifndef AUTONFUNCTIONS_HPP
#define AUTONFUNCTIONS_HPP

#include "main.h"
#include "lemlib/api.hpp"

using namespace pros;

// controller
Controller ctrl(E_CONTROLLER_MASTER);

// The first motor on the intake (5.5 motor)
Motor firstStageIntake(-18, MotorGears::green);

// The second motor on the intake (11 motor)
Motor secondStageIntake(-7);

// Ladybrown (5.5 motor)
Motor ladyBrown(-12, MotorGears::green);

// The distance sensor that detects the claw
Distance clawDistance(17);

// The color sensor that detects the color of the ring
Optical colorSensor(11);

//Ladybrown sensors
Rotation ladyLeftRot(19); // The ladybrown sensor on the left side
Rotation ladyRightRot(16); // The ladybrown sensor on the right side

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
        void autoChecks();

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
            AUTO = 0, // The claw operates in automatic mode (based on sensor readings)
        };

        /**
         * @brief Sets the state of the claw mechanism.
         *
         * @param value The desired state to set the claw to, represented by the clawState enum.
         */
        void setClawState(clawState value);

        void toggleClaw(); // Toggles the claw between open and closed states

        /**
         * @enum clawState
         * @brief Represents the possible states of a robotic claw.
         * 
         * This enumeration defines three states for controlling a robotic claw:
         * - ALLIANCE_RED: represented by the value 1.
         * - ALLIANCE_BLUE: represented by the value -1.
         * - DISABLED: represented by the value 0.
         */
        enum teamColor {
            ALLIANCE_RED = 1, // The alliance color is red
            ALLIANCE_BLUE = -1, // The alliance color is red
            DISABLED = 0 // The color sort is turned off
        };

        /**
         * @brief Sets the color for the color sort function.
         *
         * @param value The desired team color to set the sorter to look for.
         * @param velo The velocity to set the intake to when the color is detected.
         */
        void setTeamColor(teamColor value, float velo = -127);

        void toggleSort(); // Toggles the intake control between the sensor and manual control

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
        static teamColor select; // Static variable representing the team color that was selected
        static float intakeVelocity; // Static variable representing the velocity of the intake mechanism
};

extern uint8_t stage; // The stage of the motion that LB is in (0 = at rest | 1 = ready for ring | 2 = in the air)
extern void ladyBrownRoutine(); // Function for the ladybrown task
extern double targetAngle; // variable for setting the target angle of the ladybrown
extern bool detectedColor; // variable for detecting the color of the ring

#endif // AUTONFUNCTIONS_HPP