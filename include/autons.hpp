#pragma once

void default_constants();

void drive_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void motion_chaining();
void combining_movements();
void interfered_example();
void odom_drive_example();
void odom_pure_pursuit_example();
void odom_pure_pursuit_wait_until_example();
void odom_boomerang_example();
void odom_boomerang_injected_pure_pursuit_example();
void measure_offsets();

// NEW STUFF

extern uint8_t stage; // The stage of the motion that LB is in (0 = at rest | 1 = ready for ring | 2 = in the air)
extern void ladyBrownRoutine(); // Function for the ladybrown task
extern double targetAngle; // variable for setting the target angle of the ladybrown
extern bool detectedColor; // variable for detecting the color of the ring

// Initialize the detected color variable
inline bool detectedColor = false;



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
    // Constructor for the Auton_Functions class
    Auton_Functions(){}

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
    void setClawState(clawState value){ this->release = value; }

    // Toggles the claw between open and closed states
    void toggleClaw() {
      if(this->release == OPEN) this->release = CLOSE;
      else this->release = OPEN;
    }

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
    void setTeamColor(teamColor value, float velo = 127){
      this->select = value; 
      this->intakeVelocity = velo;
    }

    // Toggles the intake control between the sensor and manual control
    void toggleSort(){
      if (this->select != DISABLED) {
        oldColor = this->select;
        this->select = DISABLED;
      } else this->select = oldColor;
    }  

    /**
     * @brief 
     * Automatically controls the clamp claw mechanism.
     * This function is designed to engage or release the clamp claw
     * based on predefined autonomous routines.
     */
    void autoChecks();
  
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
      static clawState release; // Static variable representing the state of the claw release mechanism
      static teamColor select; // Static variable representing the team color that was selected
      static teamColor oldColor; // Static placeholder variable representing the team color that was selected
      static float intakeVelocity; // Static variable representing the velocity of the intake mechanism
};