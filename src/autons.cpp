#include "main.h"

using namespace pros;
/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(19.9, 0.002, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.9, 0.055, 20.25, 14.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(48_in, 30, true);
  chassis.pid_wait_until(24_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 90_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .

// Make a PID object to use to output a voltage to the motor
ez::PID PID_LB(1.21, 1.21, 0, 0, "Lady Brown"); // Create a PID object for the Lady Brown mechanism
uint8_t stage = 0; // The stage of the motion that LB is in (0 = at rest | 1 = ready for ring | 2 = in the air)
double avgPosition = 0.0; // Initialized variable that holds the avgPosition of the LadyBrown rotation sensors
double targetAngle = 0.0; // Initialized variable that holds the desired position that the LadyBrown should be at

// Function to execute the Lady Brown routine in an infinite loop
void ladyBrownRoutine() {
    
    while (true) {
        // Determine the target angle based on the current stage 
        switch (stage) {
            case 1:
                targetAngle = 44; // Set target angle for stage 1
                break;
            case 2:
                targetAngle = 540; // Set target angle for stage 2
                break;
            case 4:
                targetAngle = 660; // Set target angle for descore goal
                break;
            case 6:
                targetAngle = 320+114; // Set target angle for descore wallstake
                break;
            case 8:
                targetAngle = 360; // Set target angle for hang
                break;
            default:
                stage = 0; // Reset stage to initial
                targetAngle = 0; // Set default target angle
                break;
        }

        // Calculate the average position from two rotational sensors
        avgPosition = (ladyRot.get_position() / 100);

                  // Compute the PID output based on the current position and target angle
          // Move the Lady Brown mechanism using a PID controller
          // The movement command is scaled by 12.0
          ladyBrownMtr.move(PID_LB.compute_error(targetAngle-avgPosition,avgPosition));
        
        if(fabs(PID_LB.error) < 2.75) { // If the error is less than 2.75 degrees, stop the motor
          ladyBrownMtr.brake();
        } 


        // Delay to control the loop execution frequency
        pros::delay(25);
    }
}

Auton_Functions::clawState Auton_Functions::release = Auton_Functions::OPEN;
Auton_Functions::teamColor Auton_Functions::select = Auton_Functions::DISABLED;
Auton_Functions::teamColor Auton_Functions::oldColor = Auton_Functions::DISABLED;
float Auton_Functions::intakeVelocity = 127; // Default intake velocity

void Auton_Functions::autoChecks() {
  this->setTeamColor(DISABLED);
  while (true) {
      
    // If the claw state is OPEN, retract the claw.
    if(this->release == OPEN) { mogoClaw.retract(); }
    
    // If the claw state is CLOSE, extend the claw.
    else if(this->release == CLOSE) { mogoClaw.extend(); }

    // If the distance is within a specific range, set the claw state to CLOSE.
    else if(clawDistance.get_distance() <= 80){
      this->release = CLOSE;
    }

    if(this->select == teamColor::ALLIANCE_RED){
      if(colorSensor.get_hue() <= 254 && colorSensor.get_hue() >= 217){
        detectedColor = true;
        
        pros::delay(55);
        intake.move(-this->intakeVelocity);
        pros::delay(50);
        intake.move(this->intakeVelocity);
          
      } else detectedColor = false;
    } else if(this->select == teamColor::ALLIANCE_BLUE){
      if(colorSensor.get_hue() <= 18 && colorSensor.get_hue() >= 0){
        detectedColor = true;
        
        pros::delay(55);
        intake.move(-this->intakeVelocity);
        pros::delay(50);
        intake.move(this->intakeVelocity);

      } else detectedColor = false;
    } 

    // Delay to prevent rapid state changes.
    pros::delay(50);
  }
}

void flipDoinker(pros::adi::Pneumatics& doinker) {
  if (doinker.is_extended()) {
    doinker.retract();
  } else {
    doinker.extend();
  }
}
void Auton_Functions::useDoinker(){
  if(chassis.odom_theta_direction_get()){
    flipDoinker(rightDoinker);
  } else {
    flipDoinker(leftDoinker);
  }
}

void Auton_Functions::getCornerRings(okapi::QLength dist, float speed, bool slew){
  chassis.pid_drive_set(dist, speed, slew);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-6_in, 40, true);
  chassis.pid_wait_quick();
  intakeRaiser.extend();
  delay(1200); 
  chassis.pid_drive_set(6_in, 70, true);
  delay(500);
  intakeRaiser.retract();
  delay(400); 
  chassis.pid_drive_set(-24_in, 50, true);
  chassis.pid_wait_quick();
}

void Auton_Functions::AWP1(){
  stage = 2;
  delay(1000);
  chassis.pid_drive_set(-2_in, 75, true);
  chassis.pid_drive_set(-15_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-5_in);
  stage = 3;
  chassis.pid_wait_quick();
  chassis.pid_turn_set(54_deg, TURN_SPEED);
  chassis.pid_wait();
  this->setClawState(AUTO);
  chassis.pid_drive_set(-30_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(137_deg, 80);
  chassis.pid_wait();
  intake.move(127);
  chassis.pid_drive_set(16_in, 75, true);
  chassis.pid_wait();
  chassis.pid_turn_set(85.5_deg, TURN_SPEED);
  delay(350);
  getCornerRings(55_in, 90, true);
  chassis.pid_turn_set(275_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(34_in, 50, true);
  stage = 8;
  chassis.pid_wait();
}

void Auton_Functions::AWP2(){
  chassis.odom_theta_flip();
  AWP1();
}


void Auton_Functions::Skills(){
  // Wall Stake
  stage = 2;
  delay(900);
  chassis.pid_drive_set(-7.5_in, 75);
  chassis.pid_wait();
  delay(300);
  stage = 3;
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();


  //First Corner
  this->firstHalf_Corners();
  chassis.pid_wait();
  chassis.pid_swing_set(ez::LEFT_SWING,-95_deg,120,-15);
  chassis.pid_drive_set(3.25_in, 100, true); //aligns itself
  delay(100);
  intake.brake();
  chassis.pid_drive_set(55_in, DRIVE_SPEED,true);//drive to orgin
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  /*chassis.pid_drive_set(9_in, DRIVE_SPEED,true);
  delay(500);
  chassis.pid_turn_set(-90_deg, TURN_SPEED); //turn to second mobile goal
  delay(800);
  chassis.pid_drive_set(56_in, DRIVE_SPEED,true); //drive to orgin
  chassis.pid_wait();
  chassis.pid_turn_set(75_deg, TURN_SPEED);
  chassis.pid_wait();
*/
  // Move to reflection point
  

  //Second Corner
  chassis.odom_theta_flip();
  this->firstHalf_Corners();
  //TODO: Finished with first half of the field, now we need to do the second half
  chassis.pid_wait();
}

void wallStakeFunc(okapi::QLength dist, float speed, bool slew){
  if(dist == 0_in){
    dist = 13_in;
    speed = 227;
  } else {
    chassis.pid_drive_set(dist, speed, slew);
    chassis.pid_wait_quick();
  }
  delay(200);
  intake.brake();
  SecondStageIntake.move_relative(-100, -15);
  delay(300);
  stage = 2;
  delay(700);
  chassis.pid_drive_set(3_in - dist, fabs(100-speed), true);
  chassis.pid_wait_quick();
}

void Auton_Functions::firstHalf_Corners(){
  this->setClawState(AUTO);
  chassis.pid_drive_set(-22.1_in ,DRIVE_SPEED,true);//to mobile goal
  delay(950);
  //chassis.pid_wait_quick();
  chassis.pid_turn_set(180_deg, TURN_SPEED); //turn to ring
  chassis.pid_wait();
  intake.move(127); //starts intake
  chassis.pid_drive_set(24.5_in, DRIVE_SPEED,true); //moves to first ring
  chassis.pid_wait(); // put at the end of every movement
  chassis.pid_turn_set(-201.5_deg,TURN_SPEED); //postions toward second ring
  delay(500);
  chassis.pid_wait();
  chassis.pid_drive_set(51.5_in, 100,true); //goes to second ring
  chassis.pid_wait();
  delay(300);
  chassis.pid_turn_set(0, TURN_SPEED); // 0's out
  delay(300);
  stage = 1; //gets ready for third ring
  chassis.pid_drive_set(20.75_in,90,true); // moves to third ring
  chassis.pid_wait();
  intake.move(127);
  chassis.pid_swing_set(ez::LEFT_SWING,90_deg,100,-40);//swing manuever
  chassis.pid_turn_set(87.5_deg, TURN_SPEED); //turns to face wall stake

  chassis.pid_wait();

  wallStakeFunc(13_in, 45, true); //gets third ring scores it then moves backward
  chassis.pid_wait();
  stage = 3;
  chassis.pid_wait();
  chassis.pid_drive_set(-1.8_in,DRIVE_SPEED);//backs up from wall stake
  chassis.pid_wait();
  //chassis.pid_turn_set(0, TURN_SPEED); //faces toward 3 rings
  chassis.pid_turn_set(5_deg, TURN_SPEED); //corrects angle
  chassis.pid_wait();
  intake.move(127);
  chassis.pid_wait();
  chassis.pid_drive_set(60.25_in,100,true);//moves to the 3 rings
  chassis.pid_wait();
  chassis.pid_drive_set(-14.5_in,70,true);//backs up into last ring
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(11_in, 90, true); //moves to the last ring
  chassis.pid_wait();
  chassis.pid_swing_set(ez::LEFT_SWING,-160_deg,100,-40);
  chassis.pid_wait();
  chassis.pid_drive_set(-28_in, DRIVE_SPEED, true); //moves to corner
  delay(600);
  this->setClawState(OPEN);
  delay(300);
  chassis.pid_turn_set(-137.5_deg, TURN_SPEED);
  chassis.pid_wait();

  /*chassis.pid_drive_set(6_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  
  intake.brake();
  stage = 2;
  chassis.pid_wait();*/

  // TO DROP GOAL
  // this->setClawState(OPEN);

}


void Auton_Functions::RED_Auton::goalRush(){
  chassis.drive_brake_set(E_MOTOR_BRAKE_COAST);
  parent.useDoinker(); // Extend the left doinker
  chassis.pid_drive_set(40_in, 127, true); // Drive forward 36 inches at DRIVE_SPEED
  chassis.pid_wait();
  chassis.drive_brake_set(E_MOTOR_BRAKE_HOLD);
  chassis.pid_drive_set(-20_in, 50, true); // Drive backward 12 inches at DRIVE_SPEED
  chassis.pid_wait();
  parent.useDoinker();
  delay(300);
  chassis.pid_drive_set(-5_in, 30, true);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::RIGHT_SWING, -70_deg, 45, -15); // Swing left 90 degrees at TURN_SPEED
  firstStageIntake.move_velocity(127); // Move the first stage intake at full speed
  chassis.pid_wait();
  chassis.pid_drive_set(7_in, 30, true); // Drive forward 5 inches at DRIVE_SPEED
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-10_in, 50, true); // Drive backward 5 inches at DRIVE_SPEED
  chassis.pid_wait();
  chassis.pid_turn_set(-190_deg, 75); // Turn left 90 degrees at TURN_SPEED
  chassis.pid_wait_quick();
  parent.setClawState(AUTO); // Set the claw state to AUTO
  chassis.pid_drive_set(-13_in, 70, true); // Drive backward 5 inches at DRIVE_SPEED
  chassis.pid_wait();
  /*SecondStageIntake.move_relative(500, 127);
  chassis.pid_turn_set(-310_deg, 85); // Turn left 90 degrees at TURN_SPEED
  chassis.pid_wait_quick();
  // wallStakeFunc(10_in, 50, true); // Drive forward 5 inches at DRIVE_SPEED
  /*intake.move(127);
  parent.getCornerRings(55_in, 80, true); // Get corner rings
  delay(500);
  stage = 1;
  intake.move(127);
  chassis.pid_wait();*/
}

void Auton_Functions::RED_Auton::ringRush(){
  //Start on line at a -22.7 deg angle
  parent.useDoinker();
  chassis.pid_drive_set(40, 127, true);
  firstStageIntake.move(127);
  delay(1000);

  // check angle
  chassis.pid_turn_set(-44, TURN_SPEED);
  chassis.pid_wait_quick();
  parent.setClawState(AUTO);
  chassis.pid_drive_set(-25, 127, true);
  
  // check angle
  delay(300);
  parent.useDoinker();
  delay(1000);
  chassis.pid_swing_set(ez::RIGHT_SWING, -71.5_deg, 100, -50);
  delay(800);
  intake.move(127);
  chassis.pid_drive_set(30, 75, true);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::RIGHT_SWING, -140_deg, 100, -50);
  chassis.pid_wait_quick();

  
  // add end of awp for corner rings
  parent.getCornerRings(40_in, 90, true); // go to corner rings

  // go to start for preload
  chassis.pid_turn_set(-233_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(25, 127, true);
  chassis.pid_wait_quick();
}

void Auton_Functions::RED_Auton::centerGS(){}


void Auton_Functions::BLUE_Auton::centerGS(){}
void Auton_Functions::BLUE_Auton::goalRush(){
  
}

void Auton_Functions::BLUE_Auton::ringRush(){
  chassis.odom_theta_flip();
  
  RED_Auton::ringRush();
}