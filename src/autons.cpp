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

    // If team color is red, and detected color is blue: disable driver input and jostle intake, then enable driver
    if(this->select == teamColor::ALLIANCE_RED){
      if(colorSensor.get_hue() <= 254 && colorSensor.get_hue() >= 217){
        detectedColor = true;
        
        pros::delay(55);
        intake.move(-this->intakeVelocity);
        pros::delay(50);
        intake.move(this->intakeVelocity);
          
      } else detectedColor = false;
    } 
    // If team color is blue, and detected color is red: disable driver input and jostle intake, then enable driver
    else if(this->select == teamColor::ALLIANCE_BLUE){
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

/**
 * @brief general function that toggles the doinker's state of extended ot retracted
 * 
 * @param doinker pneumatics object
 */
void flipDoinker(pros::adi::Pneumatics& doinker) {
  if (doinker.is_extended()) {
    doinker.retract();
  } else {
    doinker.extend();
  }
}

/**
 * @brief Depending on if we are left side or right side, we make sure to use the correct doinker.
 *        If the axis is flipped then we want to toggle the right doinker, otherwise we want the left
 */
void Auton_Functions::useDoinker(){
  if(chassis.odom_theta_direction_get()){
    flipDoinker(rightDoinker);
  } else {
    flipDoinker(leftDoinker);
  }
}

/**
 * @brief General routine that will grab both of the desired colored rings in the corner
 * 
 * @param dist entry distance - distance from the robot to the corner
 * @param speed entry speed - the speed used initially to move into the corner
 * @param slew a boolean to let the robot accelerate up to speed over time
 */
void Auton_Functions::getCornerRings(okapi::QLength dist, float speed, bool slew){
  chassis.pid_drive_set(dist, speed, slew);
  intake.move(127);
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

/**
 * @brief Base function for win point auton (used for the left side of the field)
 */
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

/**
 * @brief mirrored function for win point auton (used for the right side of the field)
 */
void Auton_Functions::AWP2(){
  chassis.odom_theta_flip();
  AWP1();
}

/**
 * @brief function for Skills auton
 */
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
  
  // Move to reflection point
  chassis.pid_wait();
  chassis.pid_drive_set(8.5_in, 100, true); //aligns itself
  delay(500);
  chassis.pid_swing_set(ez::LEFT_SWING,-89_deg,100,-15);
  delay(100);
  intake.brake();
  chassis.pid_drive_set(61.5_in, DRIVE_SPEED,true);//drive to orgin
  chassis.pid_wait();
  chassis.pid_turn_set(87_deg, TURN_SPEED);
  chassis.pid_wait();
  
  /*chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.odom_xy_set(0.2_in, -7.26_in);    // Set the current position, you can start at a specific position with this
  */
 chassis.drive_sensor_reset();               // Reset drive sensors to 0
 //Second Corner
  
  chassis.odom_theta_flip();
  this->firstHalf_Corners();

  //TODO: Finished with first half of the field, now we need to do the second half
  chassis.pid_wait();
  chassis.pid_turn_set(-120_deg, TURN_SPEED); 
  chassis.pid_wait();
  chassis.pid_drive_set(58.25_in, 100, true); //moves to ring inside ladder
  chassis.pid_wait();
  chassis.pid_drive_set(24_in, 100, true); //moves to ring outside ladder
  chassis.pid_wait();
  chassis.pid_turn_set(-83_deg, TURN_SPEED); //turns to face mobile goal
  chassis.pid_wait();
  chassis.pid_drive_set(16_in, 100,true);
  chassis.pid_wait();
  chassis.pid_drive_set(60.25_in,70,true);//moves to the 3 rings
  chassis.pid_wait();
  chassis.pid_drive_set(-14.5_in,70,true);//backs up into last ring
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(11_in, 90, true); //moves to the last ring
  chassis.pid_wait();
  chassis.pid_swing_set(ez::LEFT_SWING,-170_deg,100,-40);
  chassis.pid_wait();
  chassis.pid_drive_set(-28_in, 63, true); //moves to corner
  delay(500);
  this->setClawState(OPEN);
  delay(300);
  chassis.pid_drive_set(2.5_in, 63, true);
  delay(800);
  chassis.pid_turn_set(-89_deg,70,true);
  chassis.pid_wait();
  chassis.pid_drive_set(61.5_in, 63, true);
  chassis.pid_wait();
  chassis.pid_turn_set(89_deg,70,true);
  chassis.pid_wait();
  chassis.pid_drive_set(61.5_in, 63, true);
  chassis.pid_wait();
}

/**
 * @brief General macro that will put a ring on a wallstake
 * 
 * @param dist entry distance - distance from the robot to the corner
 * @param speed entry speed - the speed used initially to move into the corner
 * @param slew a boolean to let the robot accelerate up to speed over time
 */
void wallStakeFunc(okapi::QLength dist, float speed, bool slew){
  if(dist == 0_in){ // used for driver skills
    dist = 13_in;
    speed = 227;
  } else { // used for program skills
    chassis.pid_drive_set(dist, speed, slew);
    chassis.pid_wait_quick();
  }
  delay(200);
  intake.brake();
  SecondStageIntake.move_relative(-100, -15);
  delay(300);
  stage = 2;
  delay(600);
  chassis.pid_drive_set(3_in - dist, fabs(100-speed), true);
  chassis.pid_wait_quick();
}

/**
 * @brief general function that will get 6 rings on a goal in the corner and one wall stake (used so we can mirror)
 */
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
  delay(500);
  chassis.pid_turn_set(-1, TURN_SPEED); // 0's out
  delay(300);
  chassis.pid_drive_set(20.65_in,90,true); // moves to third ring
  stage = 1; //gets ready for third ring
  chassis.pid_wait();
  intake.move(127);
  chassis.pid_swing_set(ez::LEFT_SWING,90_deg,100,-40);//swing manuever
  chassis.pid_turn_set(86.25_deg, TURN_SPEED); //turns to face wall stake

  chassis.pid_wait();

  wallStakeFunc(13_in, 45, true); //gets third ring scores it then moves backward
  chassis.pid_wait();
  stage = 3;
  chassis.pid_wait();
  chassis.pid_drive_set(-1.7_in,DRIVE_SPEED);//backs up from wall stake
  chassis.pid_wait();
  //chassis.pid_turn_set(0, TURN_SPEED); //faces toward 3 rings
  chassis.pid_turn_set(4_deg, TURN_SPEED); //corrects angle
  chassis.pid_wait();
  intake.move(127);
  chassis.pid_wait();
  chassis.pid_drive_set(60.25_in,70,true);//moves to the 3 rings
  chassis.pid_wait();
  chassis.pid_drive_set(-14.5_in,70,true);//backs up into last ring
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(11_in, 90, true); //moves to the last ring
  chassis.pid_wait();
  chassis.pid_swing_set(ez::LEFT_SWING,-170_deg,100,-40);
  chassis.pid_wait();
  chassis.pid_drive_set(-28_in, 63, true); //moves to corner
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

/**
 * @brief base function for goal rush (we didn't complete it)
 * 
 */
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

/**
 * @brief base function for ring rush (best auton we've had)
 * 
 */
void Auton_Functions::RED_Auton::ringRush(){
  //Start on line at a -22.7 deg angle
  parent.useDoinker();
  firstStageIntake.move(127);
  chassis.pid_drive_set(40, 127, true);
  delay(1000);

  // check angle
  chassis.pid_turn_set(-44, TURN_SPEED);
  chassis.pid_wait_quick();
  parent.setClawState(AUTO);
  chassis.pid_drive_set(-25, 127, true);
  
  // check angle
  delay(500);
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

/**
 * @brief These are functions that we never got to develop/ mirror 
 */


void Auton_Functions::RED_Auton::centerGS(){}
void Auton_Functions::BLUE_Auton::centerGS(){}
void Auton_Functions::BLUE_Auton::goalRush(){}

/**
 * @brief mirrored function for ring rush
 */
void Auton_Functions::BLUE_Auton::ringRush(){
  chassis.odom_theta_flip();
  
  RED_Auton::ringRush();
}