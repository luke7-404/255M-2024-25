#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "menu.hpp"

using namespace pros;

// controller
Controller ctrl(E_CONTROLLER_MASTER);

// Ladybrown (5.5 motor)
Motor ladyBrown(12, MotorGears::green);

//* Motor groups

// left motor group (all reversed) - ports 10, 20, 3 
MotorGroup leftMotors({-10, -20, -19}, MotorGearset::blue);
// right motor group - ports 15, 16, 3  
MotorGroup rightMotors({15, 16, 3}, MotorGearset::blue); 
// intake motor group - ports 14 (reversed), 11
MotorGroup intake({-14, 11});

//* PNEUMATICS
adi::Pneumatics intakeRaiser('A', false);
adi::Pneumatics mogoClaw('B', false);
adi::Pneumatics doinker('C', false);

//* SENSORS

//Ladybrown sensors

Rotation ladyLeftRot(9); // The ladybrown sensor on the left side
Rotation ladyRightRot(13); // The ladybrown sensor on the right side

// Inertial Sensor on port 10
Imu imu(6);

// The distance sensor that detects the claw
Distance clawDistance(18);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 7, not reversed
Rotation horizontalEnc(7);
// vertical tracking wheel encoder. Rotation sensor, port 8, not reversed
Rotation verticalEnc(8);

// horizontal tracking wheel. 2.75" diameter, 5.75" offset, front of the robot (positive)
lemlib::TrackingWheel horizontal(&horizontalEnc, 2.835, 2.25);
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
                            &horizontal, // horizontal tracking wheel
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

// Create the screen object
LCD_Menu menu;
char autonID;

/*
// A integer function that manages function calls for the Menu object
int MenuHandler(){
  checkPressedAuton(Menu, 30, 60);
  while(MenuON){
    // Checks if the menu is not deconstructed
    if(!Menu.isDeconstructed){
      Brain.Screen.render(true); // Render the screen (fixes flicker)

      // If less than or equal to 26, we are finding the tab buttons 
      if (Brain.Screen.yPosition() <= 26){
        checkPressedTab(Brain.Screen.xPosition(), Menu, PID_Obj, Odom_Obj, File);
      }

      // If the file was emphasized and not printed, print only once
      if (File.emphasized && !printed){
        LCD.printAt(155, 148, false, "File Emphasized!");
        printed = true;
      } 

      // If the selected tab was "Auton" check if a auton button was pressed
      if(Menu.enableAuton){
        // Stores the value of the selected autonomous program in autonSelected
        autonSelected = checkPressedAuton(Menu, Brain.Screen.xPosition(), Brain.Screen.yPosition());
        positionRobot(autonSelected);
      }
    
      Brain.Screen.render(true); // Render the screen (fixes flicker)
      task::sleep(250); // Refresh the screen every quarter of a second
    }
*/

void printMenu(){
    while(!menu.isDeconstructed){
        if(screen::touch_status().y <= 26) checkPressedTab(screen::touch_status().x, menu);
        
        if(menu.enableAuton){
            autonID = checkPressedAuton(menu, screen::touch_status().x, screen::touch_status().y);
        }

        ctrl.print(0, 2, "away: %.1f", getXDisplacement());
        delay(250);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    chassis.calibrate(); // calibrate sensors
    ladyLeftRot.reset();
    ladyRightRot.reset();
    ladyLeftRot.set_position(0);
    ladyRightRot.set_position(0);
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

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

// Make a PID object to use to output a voltage to the motor
lemlib::PID PID_LB(0.21, 0, 0, 16, false);
uint8_t stage = 0; // The stage of the motion that LB is in (0 = at rest | 1 = ready for ring | 2 = in the air)
double avgPosition = 0.0; // Initialized variable that holds the avgPosition of the LadyBrown rotation sensors
double targetAngle = 0.0; // Initialized variable that holds the desired position that the LadyBrown should be at

// A void function that run 
void ladyBrownRoutine(){
    
    while(true){
        switch (stage){
            case 1:
                targetAngle = 27.5;
                break;
            case 2:
                targetAngle = 135.5;
                break;
            default:
                stage = 0;
                targetAngle = 0.2;
                break;
        }

        double avgPosition = (ladyLeftRot.get_position()*0.01 + ladyRightRot.get_position()*0.01)/2;
        ladyBrown.move(PID_LB.update(targetAngle - avgPosition)*12.0);
        delay(25);
    }
}

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    chassis.setPose(0, 0, 270);
    chassis.swingToHeading(0, DriveSide::LEFT, 2000, {.direction = AngularDirection::CW_CLOCKWISE});
    chassis.waitUntilDone();
    intake.move(-127);
    delay(500);
    intake.brake();
    chassis.moveToPose(0, 0, 270, 10000);
}

/**
 * Runs in driver control
 */
void opcontrol() {

    // Create a background task for ladybrown
    Task LB_task(ladyBrownRoutine);

    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = ctrl.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = ctrl.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);

        //screen::print(E_TEXT_MEDIUM, 0, 2, "Angle Left: %.2f", ladyLeftRot.get_position()*0.01);
        //screen::print(E_TEXT_MEDIUM, 0, 50, "Angle Right: %.2f", ladyRightRot.get_position()*0.01);

        if (ctrl.get_digital(E_CONTROLLER_DIGITAL_L1)){         // if the top bumper is being held down  
            intake.move(127);                                   // out-take
        } else if (ctrl.get_digital(E_CONTROLLER_DIGITAL_L2)){  // if the bottom bumper is being held down
            intake.move(-127);                                  // intake
        } else intake.brake();                                  // if all else, stop

        //! CALL BACKS
        if(ctrl.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)) { stage++; } // Lady brown
        if(ctrl.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) { mogoClaw.toggle(); } // Mogo toggle
        if(ctrl.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) { doinker.toggle(); } // doinker toggle
        if(ctrl.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) { intakeRaiser.toggle(); } // intake raiser toggle

        // delay to save resources
        delay(10);
    }
}
