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

adi::Ultrasonic sonarLeft({5, 'G', 'H'}); 
adi::Ultrasonic sonarBack({5, 'E', 'F'}); 
adi::Ultrasonic sonarRight({5, 'A', 'B'}); 

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 7, not reversed
Rotation horizontalEnc(7);
// vertical tracking wheel encoder. Rotation sensor, port 8, not reversed
Rotation verticalEnc(8);

// horizontal tracking wheel. 2.75" diameter, 5.75" offset, front of the robot (positive)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, 2.25);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.25);



// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12, // 12 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis 
                              480, // drivetrain rpm is 480
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(0.35, // proportional gain (kP)
                                            0.003, // integral gain (kI)
                                            0.874, // derivative gain (kD)
                                            30, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(0.0758, // proportional gain (kP)
                                             0.00006552, // integral gain (kI)
                                             0.96, // derivative gain (kD)
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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    //menu.printAuton();
    chassis.calibrate(); // calibrate sensors
    ladyLeftRot.reset();
    ladyRightRot.reset();
    ladyLeftRot.set_position(0);
    ladyRightRot.set_position(0);
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
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    lcd::print(4, "pure pursuit finished!");
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

        screen::print(E_TEXT_MEDIUM, 0, 2, "Angle Left: %.2f", ladyLeftRot.get_position()*0.01);
        screen::print(E_TEXT_MEDIUM, 0, 50, "Angle Right: %.2f", ladyRightRot.get_position()*0.01);

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
