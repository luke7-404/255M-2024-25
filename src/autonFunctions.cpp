#include "autonFunctions.hpp"

// Set default value to release
Auton_Functions::clawState Auton_Functions::release = Auton_Functions::OPEN;

// Sets the state of the claw to the specified value.
void Auton_Functions::setClawState(clawState value){ this->release = value; }

// Automatically controls the claw based on the current state and sensor readings.
void Auton_Functions::autoClampClaw(){
    // Continuously check during the autonomous phase.
    while (competition::is_autonomous()) {
        
        // If the claw state is OPEN, retract the claw.
        if(this->release == OPEN) { mogoClaw.retract(); }
        
        // If the claw state is CLOSE, extend the claw.
        else if(this->release == CLOSE) { mogoClaw.extend(); }

        // If the distance is within a specific range, set the claw state to CLOSE.
        else if(clawDistance.get_distance() > 129 && clawDistance.get_distance() < 170){
            this->release = CLOSE;
        }

        // Delay to prevent rapid state changes.
        delay(100);
    }
}

/**
 * @brief This function completes the tasks to satisfy AWP conditions
 * 
 * @attention Left Side of drivers box
 */
void Auton_Functions::AWP1(){

    //TODO: Turn to alliance stake and get pre-load on stake
    this->setClawState(CLOSE);
    intakeRaiser.extend();
    drive.swingToHeading(100, DriveSide::LEFT, 1500);
    drive.waitUntilDone();
    intake.move(-127);
    delay(500);

    //TODO: Get stack ring on goal
    drive.moveToPose(-7, 3.25, 79, 1000);
    this->setClawState(OPEN);
    intake.move(-63.5);
    drive.waitUntilDone();
    intakeRaiser.retract();
    delay(750);
    intake.brake();
    drive.turnToHeading(225, 1000);

    //TODO: Get Goal
    this->setClawState(AUTO);
    drive.moveToPose(23.25, 38.00, 209, 2000, {.forwards = false, .minSpeed = -7.9375});

    //TODO: Turn and grab stack
    drive.turnToHeading(355, 1000, {.direction = AngularDirection::CW_CLOCKWISE});
    drive.waitUntilDone();
    intake.move(-127);
    drive.moveToPoint(17, 39, 1000, {.forwards = true});

    //TODO: Touch bar
    drive.waitUntilDone();
    drive.moveToPose(28.55, 42.69, 485.65, 1000);

}

/**
 * @brief This function completes the tasks to semi-satisfy AWP conditions (No bar touch)
 * 
 * @attention Right side of drivers box
 */
void Auton_Functions::AWP2(){

    
    //TODO: Grab goal
    this->setClawState(AUTO); // Let sensors read
    drive.moveToPose(0, -21, 0, 600, {.forwards = false, .minSpeed = -8.25}); // drive to goal
    drive.turnToHeading(319.25, 500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
    drive.moveToPoint(14, -32, 900, {.forwards = false, .minSpeed = -31.25});
    drive.waitUntilDone();
    drive.setPose(0.04, -25, 0);

    //TODO: Pick up ring stack
    intake.move(-127);
    drive.turnToHeading(-60, 900, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}); 
    drive.moveToPoint(-6, -13.96, 1750, {.forwards = true});

    //TODO: Turn to alliance ring stack
    drive.turnToHeading(87, 1750, {.direction = AngularDirection::CW_CLOCKWISE}); 
    drive.moveToPoint(14, -7, 1750, {.forwards = true});
    drive.waitUntilDone();

    //TODO: Pick up top of stack
    intakeRaiser.extend();
    intake.move(-127);
    drive.moveToPoint(24, -5.75, 1000, {.maxSpeed = 31.75});
    drive.waitUntilDone();
    intakeRaiser.retract();
    intake.set_brake_mode(motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
    delay(400);

    //TODO: drive back and drop goal
    drive.moveToPoint(14, -7, 500, {.forwards = false});
    drive.waitUntilDone();
    delay(2750);
    intake.brake();

    //TODO: Touch Bar
    drive.turnToHeading(172, 500, {.direction = AngularDirection::CW_CLOCKWISE});
    drive.moveToPose(16, -10, 172, 900, {.forwards = true, .maxSpeed = 127});
    drive.waitUntilDone();
    drive.setPose(0, 0, 0);
    drive.moveToPoint(0, 7, 400, {.forwards = true, .maxSpeed = 127});
}


void Auton_Functions::Skills(){
  
  drive.setPose(48, 12, 180); // Starting Pose
  
  //TODO: Grab bottomLeftGoal and bottomLeftRingA
  drive.moveToPose(bottomLeftGoal_POSE, 1250, {.forwards = false, .minSpeed = -63.5});
  delay(100);
  this->setClawState(AUTO);
  delay(300);
  intake.move(-127);
  drive.waitUntilDone();
  drive.turnToHeading(19, 1000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
  drive.moveToPoint(bottomLeftRingA_POINT, 300);


  //TODO: Grab bottomLeftRingB-E
  drive.waitUntilDone();
  drive.setPose(bottomLeftRingA_POINT, 0);
  drive.turnToHeading(-100, 750);
  drive.waitUntilDone();
  drive.moveToPoint(bottomLeftRingB_POINT, 1000);

  drive.turnToHeading(-190, 1000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
  drive.waitUntilDone();
  drive.setPose(bottomLeftRingB_POINT, 180);
  drive.moveToPoint(bottomLeftRingD_POINT, 1000, {.maxSpeed = 50}); // Gets rings C and D
  drive.waitUntilDone();
  drive.moveToPoint(bottomLeftRingB_POINT, 800, {.forwards = false, .minSpeed = -31.75});
  drive.waitUntilDone();
  drive.setPose(bottomLeftRingB_POINT, 180);
  drive.turnToPoint(bottomLeftRingE_POINT, 500);
  drive.moveToPoint(bottomLeftRingE_POINT, 750, {.maxSpeed = 45});

  //TODO: Put goal in bottomLeftCorner
  drive.waitUntilDone();
  drive.turnToHeading(20, 600, {.direction = AngularDirection::CW_CLOCKWISE, .maxSpeed = 64});
  drive.moveToPoint(0, 0, 600, {.forwards = false, .minSpeed = -127});
  delay(2250);
  this->setClawState(OPEN);
  //drive.setPose(10.5, 13.5, 315);

  //TODO: Grab bottomRightGoal and bottomRightRingA
  //drive.moveToPose(bottomRightGoal_POSE, 2000, {.forwards = true, .maxSpeed = 127});
  //TODO: Grab middleCenterRing and bottomRightRingB-E
  //TODO: Put goal in bottomRightCorner

  //TODO: Grab middleRightRing
  //TODO: Go to red alliance wall stake

  //TODO: Grab topRingA and topMiddleGoal
  //TODO: Grab topRingB-E and drop goal

  //TODO: Grab blueWallStakeRing
  //TODO: Put ring on blueWallStake then backup

}


//! RED FUNCTIONS

void Auton_Functions::RED_Auton::goalRush(){
  
}

void Auton_Functions::RED_Auton::ringRush(){

}


//? BLUE FUNCTIONS

void Auton_Functions::BLUE_Auton::goalRush(){
  
}

void Auton_Functions::BLUE_Auton::ringRush(){

}
