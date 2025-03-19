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
                            /*nullptr*/&horizontal, // horizontal tracking wheel
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

  /*// TODO: Turn to alliance stake and get pre-load on stake
  stage = 2;
  delay(1000);

  // TODO: Grab goal
  chassis.moveToPoint(0.66, -11, 500, {.forwards = false, .minSpeed = -7.9375});
  chassis.turnToHeading(-53.48, 500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
  stage = 3;
  chassis.waitUntilDone();
  autonFunc.setClawState(Auton_Functions::AUTO);
  chassis.moveToPose(18.75, -27.19, -53.48, 2000, {.forwards = false, .minSpeed = -68});
  chassis.waitUntilDone();
  delay(1000);

  // TODO: Grab ring
  chassis.turnToHeading(-125, 600, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
  chassis.waitUntilDone();
  intake.move(-127);
  chassis.moveToPoint(11, -45.5, 750, {.forwards = true}, false);
  chassis.waitUntilDone();
  delay(700);
  autonFunc.setClawState(Auton_Functions::OPEN);
  // chassis.turnToHeading(-402.3, 1500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false);
  chassis.turnToHeading(-350, 1500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
  chassis.waitUntilDone();
  autonFunc.setClawState(Auton_Functions::AUTO);
  chassis.moveToPoint(15.89, -50.72, 1500, {.maxSpeed = 40});



  // TODO: Grab stack
  doinker.retract();
  intakeRaiser.extend();
  chassis.moveToPose(19.40, -4, -330, 2400, {.forwards = true, .maxSpeed = 127, .minSpeed = 50});
  chassis.waitUntilDone();
  delay(300);
  intakeRaiser.retract();
  delay(500);

  // TODO: Touch bar
  chassis.moveToPose(24, -10, -278, 1000, {.forwards = false, .minSpeed = -55});
  chassis.waitUntilDone();
  delay(500);
  chassis.turnToHeading(-278, 500, {.minSpeed = 50});
  chassis.waitUntilDone();
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, 8, 300);
  stage = 2;
  delay(2000);
  intake.brake();*/

  // Make Blue and Red autonomous objects
  Auton_Functions::BLUE_Auton Blue(autonFunc);
  Auton_Functions::RED_Auton Red(autonFunc);
  // delay(2000);  

  // Switch conditional statement to choose from any of the 9 scenarios
  switch (autonID){
    
    // SKILLS
    case '1': // Flow through design because regardless if it is blue or red, the process is the same
    case 'A':
        Blue.~BLUE_Auton(); // Since it is a general function destroy objects
        Red.~RED_Auton();
      break;

    // AWP 1
    case '2': // General Function for AWP1 but add the color sorting RED
        setLB_Sensors(24); 
        stage = 1;
        autonFunc.setTeamColor(Auton_Functions::ALLIANCE_RED, -127); // Set the team color to RED
        autonFunc.AWP1();
        Blue.~BLUE_Auton(); // Since it is a general function destroy objects
        Red.~RED_Auton();
      break;

    case 'B': // General Function for AWP1 but add the color sorting BLUE
        setLB_Sensors(24); 
        stage = 1;
        autonFunc.setTeamColor(Auton_Functions::ALLIANCE_BLUE, -127); // Set the team color to BLUE
        autonFunc.AWP1();
        Blue.~BLUE_Auton(); // Since it is a general function destroy objects
        Red.~RED_Auton();
      break;

    // AWP 2
    case '3': // General Function for AWP2 but add the color sorting RED
        setLB_Sensors(24); 
        stage = 1;
        autonFunc.setTeamColor(Auton_Functions::ALLIANCE_RED, -127); // Set the team color to RED
        // autonFunc.AWP2();
        // TODO: Turn to alliance stake and get pre-load on stake
        stage = 2;
        delay(1000);

        // TODO: Grab goal
        chassis.moveToPoint(0.66, -11, 500, {.forwards = false, .minSpeed = 0});
        chassis.turnToHeading(-53.48, 500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
        stage = 3;
        chassis.waitUntilDone();
        autonFunc.setClawState(Auton_Functions::AUTO);
        chassis.moveToPose(15.75, -27.19, -53.48, 2000, {.forwards = false, .minSpeed = 30});
        chassis.waitUntilDone();
        delay(1000);

        // TODO: Grab ring
        chassis.turnToHeading(-125, 600, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
        chassis.waitUntilDone();
        intake.move(-127);
        chassis.moveToPoint(10, -45.5, 750, {.forwards = true}, false);
        chassis.waitUntilDone();
        delay(700);
        autonFunc.setClawState(Auton_Functions::OPEN);
        chassis.turnToHeading(-402.3, 1500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false);
        // chassis.turnToHeading(-350, 1500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
        chassis.waitUntilDone();
        autonFunc.setClawState(Auton_Functions::AUTO);
        chassis.moveToPoint(15.89, -50.72, 1500, {.maxSpeed = 40});

        Blue.~BLUE_Auton(); // Since it is a general function destroy objects
        Red.~RED_Auton();
      break;

    case 'C': // General Function for AWP2 but add the color sorting BLUE
        setLB_Sensors(24); 
        stage = 1;
        autonFunc.setTeamColor(Auton_Functions::ALLIANCE_BLUE, -127); // Set the team color to BLUE
        // autonFunc.AWP2();
        // TODO: Turn to alliance stake and get pre-load on stake
        stage = 2;
        delay(1000);

        // TODO: Grab goal
        chassis.moveToPoint(0.66, -11, 500, {.forwards = false, .minSpeed = 0});
        chassis.turnToHeading(-53.48, 500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
        stage = 3;
        chassis.waitUntilDone();
        autonFunc.setClawState(Auton_Functions::AUTO);
        chassis.moveToPose(15.75, -27.19, -53.48, 2000, {.forwards = false, .minSpeed = 40});
        chassis.waitUntilDone();
        delay(1000);

        // TODO: Grab ring
        chassis.turnToHeading(-125, 600, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
        chassis.waitUntilDone();
        intake.move(-127);
        chassis.moveToPoint(10, -45.5, 750, {.forwards = true}, false);
        chassis.waitUntilDone();
        delay(700);
        // autonFunc.setClawState(Auton_Functions::OPEN);
        // chassis.turnToHeading(-402.3, 1500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false);
        chassis.turnToHeading(-350, 1500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
        /*chassis.waitUntilDone();
        autonFunc.setClawState(Auton_Functions::AUTO);
        chassis.moveToPoint(15.89, -50.72, 1500, {.maxSpeed = 40});

        intakeRaiser.extend();
        chassis.moveToPose(19.40, -4, -330, 2400, {.forwards = true, .maxSpeed = 127, .minSpeed = 50});
        chassis.waitUntilDone();
        delay(300);
        intakeRaiser.retract();
        delay(500);

        // TODO: Touch bar
        chassis.moveToPose(24, -10, -278, 1000, {.forwards = false, .minSpeed = -55});
        chassis.waitUntilDone();
        delay(500);
        chassis.turnToHeading(-278, 500, {.minSpeed = 50});
        chassis.waitUntilDone();
        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, 8, 300);
        stage = 2;
        delay(2000);
        intake.brake();*/
        
        Blue.~BLUE_Auton(); // Since it is a general function destroy objects
        Red.~RED_Auton();
      break;

    case '4': //! RED Goal Rush 
        autonFunc.setTeamColor(Auton_Functions::ALLIANCE_RED, -127); // Set the team color to RED    
        Blue.~BLUE_Auton(); // Destroy object since we aren't using it
        //TODO: GET GOAL
        chassis.setBrakeMode(E_MOTOR_BRAKE_HOLD); // Set the brake mode to hold
        chassis.moveToPoint(0, 24, 1000, {.minSpeed = 30});
        firstStageIntake.move(-127);
        chassis.moveToPoint(0, 33, 750, {.minSpeed = 40});
        chassis.waitUntil(3);
        doinker.extend();
        chassis.moveToPoint(-0.2, 16.79, 1000, {.forwards = false, .maxSpeed = 127});
        chassis.waitUntilDone();
        chassis.setBrakeMode(E_MOTOR_BRAKE_COAST);
        doinker.retract();
        firstStageIntake.brake();
        autonFunc.setClawState(Auton_Functions::AUTO);
        delay(400);

        //TODO: GET GOAL #2 AND SCORE RING
        chassis.turnToHeading(136, 850, {.direction = AngularDirection::CW_CLOCKWISE}); // Turn to goal // 192 - 1250
        chassis.moveToPoint(-15.7, 25.2, 1000, {.forwards = false, .minSpeed = 30});
        chassis.waitUntilDone();
        intake.move(-127);

        chassis.turnToHeading(158.60, 1000, {.direction = AngularDirection::CW_CLOCKWISE});
        chassis.waitUntilDone();

        //TODO: GET CORNER RING
        chassis.setBrakeMode(E_MOTOR_BRAKE_HOLD);
        delay(200);
        intake.move(-63.5);
        autonFunc.setClawState(Auton_Functions::OPEN);
        intake.set_current_limit(3000, 1);

        //4500 timeout and 50 max speed initial
        chassis.moveToPoint(4.04, -17.83, 2500, {.maxSpeed = 50}, false);
        
         
        chassis.moveToPoint(6.35, -19.33, 2000, {.minSpeed = 100}, false);
        chassis.moveToPoint(4.66, -14.56, 1000, {.forwards = false}, false);
        intake.brake();
        firstStageIntake.move(-127);
        chassis.moveToPoint(8.02, -22.2, 2000, {.minSpeed = 127}, false);
        chassis.waitUntilDone();

        // TODO: GRAB GOAL
        chassis.moveToPoint(-0.87, -3.17, 1500, {.forwards = false, .maxSpeed = 30}, false);
        chassis.waitUntilDone();
        chassis.turnToHeading(204, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 30});
        autonFunc.setClawState(Auton_Functions::AUTO);
        chassis.moveToPose(7.75, 28.85, 204, 850, {.forwards = false, .minSpeed = 75}, false);
        chassis.waitUntilDone();
        intake.move(-127);
        /*delay(300);
        chassis.turnToHeading(24, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 30});*/
        
        // Red.goalRush();
      break;

    case 'D': //? BLUE Goal Rush 
        autonFunc.setTeamColor(Auton_Functions::ALLIANCE_BLUE); // Set the team color to BLUE    
        Red.~RED_Auton(); // Destroy object since we aren't using it
        // Blue.goalRush();
        //? BLUE GOAL RUSH
        chassis.setBrakeMode(E_MOTOR_BRAKE_HOLD);
        chassis.moveToPose(0.37, 33.13, 5.4, 1000, {.lead = 0.2, .minSpeed = 100});
        chassis.waitUntilDone();
        doinker.extend();
        delay(300);
        chassis.moveToPoint(0.44, 18.74, 500, {.forwards = false, .maxSpeed = 127, .minSpeed = 50, .earlyExitRange = 2});
        chassis.waitUntilDone();
        doinker.retract();
        delay(300);
        chassis.turnToHeading(40, 300, {.minSpeed = 56});
        firstStageIntake.move(-127);
        chassis.moveToPoint(9.4, 23.4, 700, {.minSpeed = 80});
        
        // Grab Goal and deposit ring

        chassis.waitUntilDone();
        delay(200);
        chassis.turnToHeading(164, 500, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 60});
        chassis.waitUntilDone();
        delay(1300);
        autonFunc.setClawState(Auton_Functions::Auton_Functions::AUTO);
        chassis.setBrakeMode(E_MOTOR_BRAKE_COAST);
        chassis.moveToPoint(8.4, 26, 1000, {.forwards = false, .maxSpeed = 70, .minSpeed = 20});
        chassis.waitUntilDone();
        firstStageIntake.brake();
        delay(150);
        chassis.setBrakeMode(E_MOTOR_BRAKE_HOLD);
        secondStageIntake.move(-127);
        chassis.moveToPoint(15.65, 2.09, 750);
        chassis.waitUntilDone();
        chassis.turnToHeading(50, 500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 50});
        chassis.waitUntilDone();
        delay(400);
        autonFunc.setClawState(Auton_Functions::OPEN);

        // Get and score second goal

        delay(300);
        intake.brake();
        chassis.turnToHeading(76.5, 300, {.maxSpeed = 80, .minSpeed = 50});
        chassis.waitUntilDone();
        chassis.moveToPose(55.6, 11, 82.8, 2000, {.lead = 0.4, .minSpeed = 70});
        chassis.waitUntil(50);
        intakeRaiser.extend();
        firstStageIntake.move(-127);
        chassis.moveToPoint(61.7, 18.4, 750);
        chassis.waitUntil(7);
        intakeRaiser.retract();
        delay(500);
        chassis.turnToHeading(128.1, 600, {.minSpeed = 30});
        chassis.waitUntilDone();
        delay(100);
        autonFunc.setClawState(Auton_Functions::AUTO);
        chassis.setBrakeMode(E_MOTOR_BRAKE_COAST);
        chassis.moveToPoint(38.7, 28.7, 1250, {.forwards = false, .maxSpeed = 60, .minSpeed = 15}, false);
        chassis.waitUntilDone();
        secondStageIntake.move(-127);
        chassis.moveToPose(13.8, 34.3, 87.8, 2000, {.forwards = false, .maxSpeed = 45}, false);
        chassis.turnToHeading(-4.5, 1000);
      break;

    case '5': //! RED Ring Rush
        Blue.~BLUE_Auton(); // Destroy object since we aren't using it
        Red.ringRush();
      break;

    case 'E': //? BLUE Ring Rush
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
