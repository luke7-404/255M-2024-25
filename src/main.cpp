// Pros/ lemlib
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// My headers
#include "menu.hpp"
#include "autonFunctions.hpp"

using namespace pros;


//* Motor groups

// left motor group (all reversed) - ports 10, 19, 20 
MotorGroup leftMotors({-2, -6, -20}, MotorGearset::blue);
// right motor group - ports 15, 16, 3  
MotorGroup rightMotors({1, 10, 14}, MotorGearset::blue); 
// intake motor group - ports 14 (reversed), 13
MotorGroup intake({-18, -7});

// Vector to hold references to all motors
std::vector<std::reference_wrapper<AbstractMotor>> motorArray = {ladyBrown, leftMotors, rightMotors, intake};

//* PNEUMATICS
adi::Pneumatics mogoClaw('A', false);
adi::Pneumatics intakeRaiser('B', false);
adi::Pneumatics doinker('C', false);

//* SENSORS

// Inertial Sensor on port 10
Imu imu(21);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 7, not reversed
Rotation horizontalEnc(-9);
// vertical tracking wheel encoder. Rotation sensor, port 8, not reversed
Rotation verticalEnc(-15);

// horizontal tracking wheel. 2.75" diameter, 5.75" offset, front of the robot (positive)
lemlib::TrackingWheel horizontal(&horizontalEnc, 2.835, -2.25);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, 2.835, 0);



// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12, // 12 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis 
                              480, // drivetrain rpm is 480
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(5.75, // proportional gain (kP)
                                            0.0, // integral gain (kI)
                                            2.05, // derivative gain (kD)
                                            30, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.5, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             10, // anti windup
                                             0.5, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             1, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr /*&horizontal*/, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.043 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.043 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// Create a data file object for storing and retrieving data
file::data_File dataFile(&chassis, motorArray);

// Create the screen object for the LCD menu
LCD_Menu menu(&dataFile);

// Create an object for autonomous functions
Auton_Functions autonFunc(chassis, intakeRaiser, mogoClaw, doinker, intake);

// Variable to store the autonomous routine ID
char autonID;

// void function that handles screen interactions
void printMenu(){
    // while the menu object is able to function
    while(!menu.isDeconstructed){
        autonID = menu.getAutonID();
        ctrl.print(0, 0, "ID: %c: %s", autonID, menu.getAutonName());
        ctrl.clear_line(0);
        delay(250); // save computer resources
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
void setLB_Sensors(float position){
  ladyLeftRot.set_position(position/ 0.01);
  ladyRightRot.set_position(position/ 0.01);  
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    // calibrate sensors
    chassis.calibrate(); 
    ladyLeftRot.reset();
    ladyRightRot.reset();
    setLB_Sensors(0.0);
    stage = 0;
    colorSensor.set_led_pwm(100); // Set the color sensor LED brightness

    // Create a background task for ladybrown
    Task LB_task(ladyBrownRoutine);
    Task autoClamp(autoHandler); // Create a background task for the autoCheck function

    // start tasks 
    menu.init();
    Task printGUI(printMenu);
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}


/**
 * Runs during autonomous
 */
void autonomous() {

  setLB_Sensors(24); 
  stage = 1;

  double g1x = 0; //8
  double g1y = 0; // 3

  stage = 2; // Alliance wall stake
  delay(500);
  autonFunc.setTeamColor(Auton_Functions::DISABLED); // turn off color sensor
  chassis.setBrakeMode(E_MOTOR_BRAKE_HOLD); // Set the brake mode to hold
  chassis.moveToPoint(0, -5, 500, {.forwards = false, .maxSpeed = 127, .minSpeed = 40});
  chassis.turnToHeading(-90, 500);
  autonFunc.setClawState(Auton_Functions::AUTO);
  chassis.moveToPoint(18.7, -10.5, 650, {.forwards = false, .minSpeed = 30}); // grab goal
  stage = 3;
  chassis.waitUntilDone();
  chassis.turnToHeading(-175.85, 300);
  chassis.waitUntilDone();
  chassis.moveToPose(20-g1x,-30.56+g1y, -179, 1500, {.minSpeed = 40}, false); // tune #1 First ring // 20 // 17
  intake.move(-127);
  chassis.moveToPose(37-g1x, -77+g1y, -177.88, 2250, {.minSpeed = 50}, false); // tune #2 Second ring // 37 //34
  delay(800);
  chassis.turnToHeading(-353.45, 750, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false); // tune #3 turn to corner rings
  
  // g1y = 6;

  chassis.moveToPose(45.5-g1x, 6-g1y, -353, 4000, {.maxSpeed = 80, .minSpeed = 40}, false); // tune #4 get the three corner rings // 45.5
  
  // get the last corner ring
  chassis.moveToPoint(42.1-g1x, -20-g1y, 1250, {.forwards = false, .minSpeed = 30}, false); // move back // 42.1
  chassis.turnToHeading(-327, 400, {.direction = AngularDirection::CW_CLOCKWISE, .maxSpeed = 35}, false); // turn to ring
  chassis.moveToPoint(49-g1x, -12-g1y, 1000, {.maxSpeed = 50}, false); // move to last ring // 49 // 

  // get in the corner
  delay(500);
  chassis.turnToHeading(-153, 500, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 30}, false); // turn to corner
  
  chassis.moveToPoint(57.72-g1x, 3-g1y, 1500, {.forwards = false, .maxSpeed = 70}, false); // push into corner
  intake.brake(); // stop intake
  autonFunc.setClawState(Auton_Functions::OPEN); // open claw


  // get second goal
  chassis.waitUntilDone();
  delay(1000);

  
  chassis.turnToHeading(-117, 300, {}, false); // turn to middle 
  chassis.moveToPose(6-g1x, -5-g1y, -88, 5000, {.lead = 0.6, .minSpeed = 30}, false); // move to middle
  autonFunc.setClawState(Auton_Functions::AUTO); // set claw to auto

  double g2 = 1.5;

  chassis.turnToHeading(90, 500, {}, false); // turn to goal
  chassis.moveToPoint(g2-20, -6, 1000, {.forwards = false, .maxSpeed = 60}, false);
  // chassis.moveToPose(g2-22, -4, 90, 7500, {.forwards = false, .lead = 0.15, .minSpeed = 40}, false); // push into goal

  // get the first ring of the second goal
  delay(500); 
  chassis.turnToHeading(177, 500, {.minSpeed = 30}); // turn to first ring
  chassis.waitUntilDone();
  chassis.moveToPoint(g2-17.9, -17.3, 1000, {.minSpeed = 30}, false); // move to first ring
  intake.move(-127); // intake ring
  chassis.moveToPose(g2-39, -70, 177.88, 3500, {.minSpeed = 50}, false); // move to second ring
  delay(800);
  chassis.turnToHeading(353, 750, {.direction = AngularDirection::CW_CLOCKWISE}, false); // tune #3 turn to corner rings
  chassis.moveToPose(g2-44.85, 6.5, 355.1, 15000, {.lead = 0.35, .maxSpeed = 80, .minSpeed = 40}, false); // tune #4 get the three corner rings
  
  // get the last corner ring
  chassis.moveToPoint(g2-43.10, -18.75, 1000, {.forwards = false, .minSpeed = 30}, false); // move back
  chassis.turnToHeading(321.32, 700, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 20}, false); // turn to ring
  chassis.moveToPoint(g2-49, -12, 1000, {.maxSpeed = 50}, false); // move to last ring

  // get in the corner
  delay(500);
  chassis.turnToHeading(153, 500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 30}, false); // turn to corner
  
  chassis.moveToPoint(g2-58.25, 3, 1500, {.forwards = false, .maxSpeed = 70}, false); // push into corner
  intake.brake(); // stop intake
  autonFunc.setClawState(Auton_Functions::OPEN); // open claw

  // Start third goal
  delay(500);
  chassis.waitUntilDone();
  firstStageIntake.move(-127); // intake ring
  chassis.moveToPoint(g2-2.35, -47, 3000, {.maxSpeed = 50}, false); // move to first ring
  chassis.waitUntilDone();
  delay(200);
  secondStageIntake.move_relative(-650, -63); // move ring
  delay(1000);

  chassis.moveToPoint(28.5-g2, -71, 2500, {.maxSpeed = 70}, false); // move to second ring
  chassis.waitUntilDone();
  firstStageIntake.brake();
  chassis.turnToHeading(43.95, 900, {}, false); // turn to goal
  autonFunc.setClawState(Auton_Functions::AUTO); // set claw to auto

  // TODO: TUNE GRAB
  chassis.moveToPoint(2.08, -98.23, 1000, {.forwards = false, .maxSpeed = 60, .minSpeed = 30}, false); // push into goal
  intake.move(-127);
  delay(1000);
  chassis.turnToHeading(-43.2, 500, {}, false); // turn to third ring
  
  // TODO: TUNE MOVE
  chassis.moveToPoint(-8.7, -73.97, 1000, {.maxSpeed = 63.5}, false);
  chassis.moveToPoint(-31.59, -45, 1300, {.maxSpeed = 63.5}, false); // move to third ring and set up for fourth and fifth 
  chassis.turnToHeading(-176.67, 1000, {}, false);

  autonFunc.setTeamColor(Auton_Functions::ALLIANCE_RED, -127); // Set the team color to red
  chassis.moveToPoint(-36.58, -97.35, 2000, {.maxSpeed = 80}); // move to fourth and fifth ring 

  // get the last corner ring
  chassis.moveToPoint(g2-40.94, -87.9, 1000, {.forwards = false, .minSpeed = 30}, false); // move back
  chassis.turnToHeading(-139.14, 700, {.minSpeed = 20}, false); // turn to ring
  chassis.moveToPoint(g2-47.97, -94.24, 1000, {.maxSpeed = 50}, false); // move to last ring

  // get in the corner
  delay(750);
  intake.brake();
  firstStageIntake.move(-127);
  chassis.moveToPoint(g2-51.72, -98.42, 500, {}, false); // hold blue
  chassis.moveToPoint(g2-47.97, -94.24, 600, {.forwards = false}, false); // move back
  chassis.turnToHeading(27.23, 500, {.minSpeed = 30}, false); // turn to corner
  autonFunc.setClawState(Auton_Functions::OPEN);// open claw
  chassis.moveToPoint(g2-57.23, -109.03, 2000, {.forwards = false, .minSpeed = 30}, false); // push into corner
  intake.brake(); // stop intake

  // Get fourth goal
  delay(200);
  chassis.turnToHeading(84.98, 500, {}, false);
  intake.move(-127);
  chassis.moveToPoint(g2-8.19, -99.81, 800, {.minSpeed = 15}, false); // push with intake
  intake.move(127);
  chassis.moveToPoint(17.94-g2, -108.15, 1750, {.minSpeed = 30}, false);
  chassis.moveToPoint(53.79-g2, -120, 1000, {.minSpeed = 40}, false);
  delay(150);
  /*chassis.moveToPoint(32.68, -120.39, 500, {.forwards = false}, false);
  chassis.turnToHeading(51.13, 500);
  stage = 1;
  intake.move(-127);
  chassis.moveToPoint(46.98, -108.68, 500, {}, false);
  chassis.moveToPoint(2.73, -110.88, 1000, {.forwards = false}, false);* /
*/

}

/**
 * Runs in driver control
 */
void opcontrol() {
  bool macro = true;
  stage = 3; // Set the stage to 3 to reset the LadyBrown
  autonFunc.setTeamColor(Auton_Functions::DISABLED);

  // controller
  // loop to continuously update motors
  while (true) {
    // get joystick positions
    int leftY = ctrl.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = ctrl.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
    // move the chassis with curvature drive
    chassis.arcade(leftY, rightX);

    if(!detectedColor) { // If the color sensor hasn't detected a color
      if (ctrl.get_digital(E_CONTROLLER_DIGITAL_L1)){         // if the top bumper is being held down  
          intake.move(127);                                   // out-take
      } else if (ctrl.get_digital(E_CONTROLLER_DIGITAL_L2)){  // if the bottom bumper is being held down
          intake.move(-127);                                  // intake
      } else if (ctrl.get_digital(E_CONTROLLER_DIGITAL_R1)) { // if the top right bumper is being held down
          firstStageIntake.move(-127);                        // intake with the first stage
      } else intake.brake();                                  // if all else, stop
    }

    //! CALL BACKS
    if(ctrl.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)) { stage++; } // Lady brown
    if(ctrl.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) { autonFunc.toggleClaw(); } // claw toggle
    if(ctrl.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) { doinker.toggle(); } // doinker toggle
    if(ctrl.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) { intakeRaiser.toggle(); } // intake raiser toggle
    if(ctrl.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)) { autonFunc.toggleSort(); } // claw auto color sort toggle
    if (ctrl.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT) && macro) { // Macro for skills
      chassis.setPose(0, 0, 0);
      macro = false;
      chassis.moveToPose(0, chassis.getPose().y-8, 0, 600, {}, false);
      stage = 2;
    }
    if(ctrl.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)) { // Lady brown for hang 
      ladyBrown.set_brake_mode(motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
      stage = 4; 
    } 

    // delay to save resources
    delay(10);
  }
}
