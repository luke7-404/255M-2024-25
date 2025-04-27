#include "menu.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {8, -5, -10},     // Left Chassis Ports (negative port will reverse it!)
    {-3, 4, 9},  // Right Chassis Ports (negative port will reverse it!)

    18,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM = cartridge * (motor gear / wheel gear)

Auton_Functions autonFunc; // Auton function object

LCD_Menu menu; // LCD Menu object

// Variable to store the autonomous routine ID
char autonID;

// void function that handles screen interactions
void printMenu(){
    // while the menu object is able to function
    while(!menu.isDeconstructed){
        autonID = menu.getAutonID();
        master.print(0, 0, "ID: %c: %s", autonID, menu.getAutonName());
        master.clear_line(0);
        pros::delay(250); // save computer resources
    }
}

// void function that handles calling the autoChecks function
void autoHandler(){
  autonFunc.autoChecks();
}

/**
 * @brief Set the LB Sensors object
 * 
 * @param position the position to set the sensors to
 */
void setLB_Sensor(float position){
  ladyRot.set_position(position * 100);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  // chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  // chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.25);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(1.043, 1.043);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);


  // Initialize chassis and auton selector
  chassis.initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
  setLB_Sensor(0.0);
  stage = 0;
  colorSensor.set_led_pwm(100); // Set the color sensor LED brightness
  ladyBrownMtr.set_brake_mode(MOTOR_BRAKE_HOLD); // Set the lady brown motor to hold mode
  // Create a background task for ladybrown
  pros::Task LB_task(ladyBrownRoutine);
  pros::Task autoClamp(autoHandler); // Create a background task for the autoCheck function

  // start tasks 
  menu.init();
  pros::Task printGUI(printMenu);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  // Make Blue and Red autonomous objects
  Auton_Functions::BLUE_Auton Blue(autonFunc);
  Auton_Functions::RED_Auton Red(autonFunc);
  // delay(2000);  

  autonID = 'A';
  float x = 45.0; // Set the initial position of the lady brown sensor to 0.0

  // Switch conditional statement to choose from any of the 9 scenarios
  switch (autonID){
    
    // SKILLS
    case '1': // Flow through design because regardless if it is blue or red, the process is the same
    case 'A':
        setLB_Sensor(x); 
        stage = 1;
        autonFunc.setTeamColor(Auton_Functions::ALLIANCE_RED); 
        autonFunc.Skills();
        Blue.~BLUE_Auton(); // Since it is a general function destroy objects
        Red.~RED_Auton();
      break;

    case '2': //! Center GS RED
        autonFunc.setTeamColor(Auton_Functions::ALLIANCE_RED); // Set the team color to RED
        Blue.~BLUE_Auton(); // Since it is a general function destroy objects
        Red.centerGS();
      break;

    case 'B': //? Center GS RED
        autonFunc.setTeamColor(Auton_Functions::ALLIANCE_BLUE); // Set the team color to BLUE
        Red.~RED_Auton();
        Blue.centerGS(); 
      break;

    // AWP 1
    case '3': // General Function for AWP1 but add the color sorting RED
        setLB_Sensor(x); 
        stage = 1;
        autonFunc.setTeamColor(Auton_Functions::ALLIANCE_RED); // Set the team color to RED
        autonFunc.AWP1();
        Blue.~BLUE_Auton(); // Since it is a general function destroy objects
        Red.~RED_Auton();
      break;

    case 'C': // General Function for AWP1 but add the color sorting BLUE
        setLB_Sensor(x); 
        stage = 1;
        autonFunc.setTeamColor(Auton_Functions::ALLIANCE_BLUE); // Set the team color to BLUE
        autonFunc.AWP1();
        Blue.~BLUE_Auton(); // Since it is a general function destroy objects
        Red.~RED_Auton();
      break;

    // AWP 2
    case '4': // General Function for AWP2 but add the color sorting RED
        setLB_Sensor(x); 
        stage = 1;
        autonFunc.setTeamColor(Auton_Functions::ALLIANCE_RED); // Set the team color to RED
        autonFunc.AWP2();
        Blue.~BLUE_Auton(); // Since it is a general function destroy objects
        Red.~RED_Auton();
      break;

    case 'D': // General Function for AWP2 but add the color sorting BLUE
        setLB_Sensor(x); 
        stage = 1;
        autonFunc.setTeamColor(Auton_Functions::ALLIANCE_BLUE); // Set the team color to BLUE
        autonFunc.AWP2();
        Blue.~BLUE_Auton(); // Since it is a general function destroy objects
        Red.~RED_Auton();
      break;

    case '5': //! RED Goal Rush 
        autonFunc.setTeamColor(Auton_Functions::ALLIANCE_RED); // Set the team color to RED    
        Blue.~BLUE_Auton(); // Destroy object since we aren't using it
        Red.goalRush();
      break;

    case 'E': //? BLUE Goal Rush 
        autonFunc.setTeamColor(Auton_Functions::ALLIANCE_BLUE); // Set the team color to BLUE    
        Red.~RED_Auton(); // Destroy object since we aren't using it
        Blue.goalRush();
      break;

    case '6': //! RED Ring Rush
        autonFunc.setTeamColor(Auton_Functions::ALLIANCE_RED); // Set the team color to RED    
        Blue.~BLUE_Auton(); // Destroy object since we aren't using it
        Red.ringRush();
      break;

    case 'F': //? BLUE Ring Rush
        autonFunc.setTeamColor(Auton_Functions::ALLIANCE_BLUE); // Set the team color to BLUE    
        Red.~RED_Auton(); // Destroy object since we aren't using it
        Blue.ringRush();
      break;
      
    default: // If nothing ran destroy objects
        Blue.~BLUE_Auton();
        Red.~RED_Auton();
      break;
  }

}


/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    //if (master.get_digital_new_press(DIGITAL_X))
      //chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  bool macro = true;
  stage = 3;
  autonFunc.setTeamColor(Auton_Functions::DISABLED);

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade

    // . . .
    // Put more user control code here!
    // . . .

    if(!detectedColor) { // If the color sensor hasn't detected a color
      if (master.get_digital(DIGITAL_L1)){         // if the top bumper is being held down  
        intake.move(-127);                         // out-take
      } else if (master.get_digital(DIGITAL_L2)){  // if the bottom bumper is being held down
        intake.move(127);                          // intake
      } else intake.brake();                       // if all else, stop
    }

    //! CALL BACKS
    if(master.get_digital_new_press(DIGITAL_A)) { autonFunc.toggleClaw(); } // claw toggle
    if(master.get_digital_new_press(DIGITAL_X)) { leftDoinker.toggle(); } // doinker toggle
    if(master.get_digital_new_press(DIGITAL_B)) { rightDoinker.toggle(); } // doinker toggle
    if(master.get_digital_new_press(DIGITAL_Y)) { intakeRaiser.toggle(); } // intake raiser toggle

    if(master.get_digital_new_press(DIGITAL_R2)) { stage++; } // Lady brown
    if(master.get_digital_new_press(DIGITAL_R1)) { stage = 6; } // descore wall stake
    if(master.get_digital_new_press(DIGITAL_DOWN)) { stage = 4; } // tip goal
    
    if(master.get_digital_new_press(DIGITAL_LEFT)) { autonFunc.toggleSort(); } // toggle color sort
    if(master.get_digital_new_press(DIGITAL_UP)) { 
      chassis.pid_targets_reset();                // Resets PID targets to 0
      chassis.drive_imu_reset();                  // Reset gyro position to 0
      chassis.drive_sensor_reset();               // Reset drive sensors to 0
      pros::delay(250);
      wallStakeFunc(0_in,0, false); 
    } // toggle color sort

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
