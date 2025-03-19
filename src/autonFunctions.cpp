#include "autonFunctions.hpp"

// Set default value to release
Auton_Functions::clawState Auton_Functions::release = Auton_Functions::OPEN;

// Sets the state of the claw to the specified value.
void Auton_Functions::setClawState(clawState value){ this->release = value; }

// Toggles the claw between open and closed states.
void Auton_Functions::toggleClaw(){
    if(this->release == OPEN) this->release = CLOSE;
    else this->release = OPEN;
}

// Set default value to release
Auton_Functions::teamColor Auton_Functions::select = Auton_Functions::DISABLED;

// A placeholder for the team color
Auton_Functions::teamColor oldColor = Auton_Functions::DISABLED;

// initialize the intake velocity
float Auton_Functions::intakeVelocity = 0.0;

// Sets the color of the team to the specified value.
void Auton_Functions::setTeamColor(teamColor value, float velo){ 
    this->select = value; 
    this->intakeVelocity = velo;
}

// Toggles the color between the selected color and disable.
void Auton_Functions::toggleSort(){
    if (this->select != DISABLED) {
        oldColor = this->select;
        this->select = DISABLED;
    } else this->select = oldColor;
}

// Initialize the detected color variable
bool detectedColor = false;

// Automatically controls the claw and intake based on the current state and sensor readings.
void Auton_Functions::autoChecks(){

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
            if(colorSensor.get_hue() <= 303 && colorSensor.get_hue() >= 217){
                detectedColor = true;
                
                delay(55);
                intake.move(-this->intakeVelocity);
                delay(50);
                intake.move(this->intakeVelocity);
                
            } else detectedColor = false;
        } else if(this->select == teamColor::ALLIANCE_BLUE){
            if(colorSensor.get_hue() <= 18 && colorSensor.get_hue() >= 0){
                detectedColor = true;
                
                delay(55);
                intake.move(-this->intakeVelocity);
                delay(50);
                intake.move(this->intakeVelocity);

            } else detectedColor = false;
        } 

        // Delay to prevent rapid state changes.
        delay(50);
    }
}

/**
 * @brief This function completes the tasks to satisfy AWP conditions
 * 
 * @attention Left Side of drivers box
 */
void Auton_Functions::AWP1(){
    
    // TODO: Turn to alliance stake and get pre-load on stake
    stage = 2;
    delay(1000);

    // TODO: Grab goal
    drive.moveToPoint(-2.81, -14, 500, {.forwards = false, .minSpeed = -7.9375});
    drive.turnToHeading(56.7, 750);
    this->setClawState(Auton_Functions::AUTO);
    stage = 3;
    drive.moveToPose(-24, -30, 58, 2000, {.forwards = false, .minSpeed = -65});
    
    // TODO: Grab ring
    drive.waitUntilDone();
    drive.turnToHeading(123, 600);
    intake.move(-127);
    drive.moveToPoint(-16.5, -36.5, 1000, {.forwards = true, .minSpeed = 30});

    // TODO: Grab stack
    drive.waitUntilDone();
    delay(1000);
    drive.turnToHeading(356, 650);
    intakeRaiser.extend();
    drive.moveToPoint(-17.5, 8.6, 1250, {.forwards = true, .maxSpeed = 55, .minSpeed = 10}, false);
    drive.waitUntilDone();
    intakeRaiser.retract();

    // TODO: Touch bar
    delay(1000);
    drive.moveToPose(-13, -3.3, -110.0, 2000, {.forwards = false, .maxSpeed = 40});
    drive.waitUntilDone();
    drive.setPose(0, 0, 0);
    drive.moveToPoint(0, 18, 300);
    stage = 2;
    delay(2000);
    intake.brake();
}

/**
 * @brief This function completes the tasks to semi-satisfy AWP conditions (No bar touch)
 * 
 * @attention Right side of drivers box
 */
void Auton_Functions::AWP2(){
    // TODO: Turn to alliance stake and get pre-load on stake
    stage = 2;
    delay(1000);

    // TODO: Grab goal
    drive.moveToPoint(0.66, -11, 500, {.forwards = false, .minSpeed = -7.9375});
    drive.turnToHeading(-53.48, 500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
    stage = 3;
    drive.waitUntilDone();
    this->setClawState(Auton_Functions::AUTO);
    drive.moveToPose(18.75, -27.19, -53.48, 2000, {.forwards = false, .minSpeed = -68});
    drive.waitUntilDone();
    delay(1000);

    // TODO: Grab ring
    drive.turnToHeading(-125, 600, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
    drive.waitUntilDone();
    intake.move(-127);
    drive.moveToPoint(11, -45.5, 750, {.forwards = true}, false);
    drive.waitUntilDone();
    delay(700);
    this->setClawState(OPEN);
    drive.turnToHeading(-402.3, 1500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false);
    // drive.turnToHeading(-350, 1500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
    drive.waitUntilDone();
    this->setClawState(AUTO);
    drive.moveToPoint(15.89, -50.72, 1500, {.maxSpeed = 40});



    // TODO: Grab stack
    /*doinker.retract();
    intakeRaiser.extend();
    drive.moveToPose(19.40, -4, -330, 2400, {.forwards = true, .maxSpeed = 127, .minSpeed = 50});
    drive.waitUntilDone();
    delay(300);
    intakeRaiser.retract();
    delay(500);

    // TODO: Touch bar
    drive.moveToPose(24, -10, -278, 1000, {.forwards = false, .minSpeed = -55});
    drive.waitUntilDone();
    delay(500);
    drive.turnToHeading(-278, 500, {.minSpeed = 50});
    drive.waitUntilDone();
    drive.setPose(0, 0, 0);
    drive.moveToPoint(0, 8, 300);
    stage = 2;
    delay(2000);
    intake.brake();*/
}


void Auton_Functions::Skills(){
  double g1x = 0; //8
  double g1y = 0; // 3

  stage = 2; // Alliance wall stake
  delay(500);
  this->setTeamColor(Auton_Functions::DISABLED); // turn off color sensor
  drive.setBrakeMode(E_MOTOR_BRAKE_HOLD); // Set the brake mode to hold
  drive.moveToPoint(0, -5, 500, {.forwards = false, .maxSpeed = 127, .minSpeed = 30});
  drive.turnToHeading(-90, 500);
  this->setClawState(Auton_Functions::AUTO);
  drive.moveToPoint(18.7, -10.5, 650, {.forwards = false, .minSpeed = 30}); // grab goal
  stage = 3;
  drive.waitUntilDone();
  drive.turnToHeading(-175.85, 300);
  drive.waitUntilDone();
  drive.moveToPose(20-g1x,-30.56+g1y, -179, 1500, {.minSpeed = 40}, false); // tune #1 First ring // 20 // 17
  intake.move(-127);
  drive.moveToPose(37-g1x, -77+g1y, -177.88, 2250, {.minSpeed = 50}, false); // tune #2 Second ring // 37 //34
  delay(800);
  drive.turnToHeading(-353.45, 750, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false); // tune #3 turn to corner rings
  
  // g1y = 6;

  drive.moveToPose(45.5-g1x, 6-g1y, -353, 4000, {.maxSpeed = 80, .minSpeed = 40}, false); // tune #4 get the three corner rings // 45.5
  
  // get the last corner ring
  drive.moveToPoint(42.1-g1x, -20-g1y, 1250, {.forwards = false, .minSpeed = 30}, false); // move back // 42.1
  drive.turnToHeading(-327, 400, {.direction = AngularDirection::CW_CLOCKWISE, .maxSpeed = 35}, false); // turn to ring
  drive.moveToPoint(49-g1x, -12-g1y, 1000, {.maxSpeed = 50}, false); // move to last ring // 49 // 

  // get in the corner
  delay(500);
  drive.turnToHeading(-153, 500, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 30}, false); // turn to corner
  
  drive.moveToPoint(57.72-g1x, 3-g1y, 1500, {.forwards = false, .maxSpeed = 70}, false); // push into corner
  intake.brake(); // stop intake
  this->setClawState(Auton_Functions::OPEN); // open claw


  // get second goal
  drive.waitUntilDone();
  delay(1000);

  
  drive.turnToHeading(-117, 300, {}, false); // turn to middle 
  drive.moveToPose(6-g1x, -5-g1y, -88, 5000, {.lead = 0.6, .minSpeed = 30}, false); // move to middle
  this->setClawState(Auton_Functions::AUTO); // set claw to auto

  double g2 = 0/*8.5*/;

  drive.turnToHeading(90, 500, {}, false); // turn to goal
  drive.moveToPose(g2-22, -2, 90, 9000, {.forwards = false, .lead = 0.15, .minSpeed = 30}, false); // push into goal

  // get the first ring of the second goal
  delay(500); 
  drive.turnToHeading(177, 500, {.minSpeed = 30}); // turn to first ring
  drive.waitUntilDone();
  drive.moveToPoint(g2-17.9, -17.3, 1000, {.minSpeed = 30}, false); // move to first ring
  intake.move(-127); // intake ring
  drive.moveToPose(g2-39, -70, 177.88, 3500, {.minSpeed = 50}, false); // move to second ring
  delay(800);
  drive.turnToHeading(353, 750, {.direction = AngularDirection::CW_CLOCKWISE}, false); // tune #3 turn to corner rings
  drive.moveToPose(g2-44.85, 6.5, 355.1, 15000, {.lead = 0.35, .maxSpeed = 80, .minSpeed = 40}, false); // tune #4 get the three corner rings
  
  // get the last corner ring
  drive.moveToPoint(g2-43.10, -18.75, 1000, {.forwards = false, .minSpeed = 30}, false); // move back
  drive.turnToHeading(321.32, 700, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 20}, false); // turn to ring
  drive.moveToPoint(g2-49, -12, 1000, {.maxSpeed = 50}, false); // move to last ring

  // get in the corner
  delay(500);
  drive.turnToHeading(153, 500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 30}, false); // turn to corner
  
  drive.moveToPoint(g2-58.25, 3, 1500, {.forwards = false, .maxSpeed = 70}, false); // push into corner
  intake.brake(); // stop intake
  this->setClawState(Auton_Functions::OPEN); // open claw

  // Start third goal
  delay(500);
  drive.waitUntilDone();
  firstStageIntake.move(-127); // intake ring
  drive.moveToPoint(-2.35, -47, 3000, {.maxSpeed = 50}, false); // move to first ring
  drive.waitUntilDone();
  delay(200);
  secondStageIntake.move_relative(-500, -63); // move ring
  delay(1000);

  drive.moveToPoint(28.5, -71, 2500, {.maxSpeed = 70}, false); // move to second ring
  drive.waitUntilDone();
  firstStageIntake.brake();
  drive.turnToHeading(48, 500, {}, false); // turn to goal
  this->setClawState(Auton_Functions::AUTO); // set claw to auto

  // TODO: TUNE GRAB
  drive.moveToPoint(5, -92, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 30}, false); // push into goal
  intake.move(-127);
  delay(1000);
  drive.turnToHeading(-43.2, 500, {}, false); // turn to third ring
  
  // TODO: TUNE MOVE
  drive.moveToPoint(-40.15, -49.74, 1300, {.maxSpeed = 63.5}, false); // move to third ring and set up for fourth and fifth 
  drive.turnToHeading(-177, 500, {}, false);

  this->setTeamColor(Auton_Functions::ALLIANCE_RED, -127); // Set the team color to red
  drive.moveToPoint(-42.84, -105.31, 2000, {.maxSpeed = 80}); // move to fourth and fifth ring 

  // get the last corner ring
  drive.moveToPoint(-40.94, -87.9, 1000, {.forwards = false, .minSpeed = 30}, false); // move back
  drive.turnToHeading(-139.14, 700, {.minSpeed = 20}, false); // turn to ring
  // intakeRaiser.extend();
  drive.moveToPoint(-47.97, -94.24, 1000, {.maxSpeed = 50}, false); // move to last ring
  // intakeRaiser.retract();

  // get in the corner
  delay(750);
  intake.brake();
  firstStageIntake.move(-127);
  drive.moveToPoint(-51.72, -98.42, 500, {}, false); // hold blue
  drive.moveToPoint(-47.97, -94.24, 600, {.forwards = false}, false); // move back
  drive.turnToHeading(27.23, 500, {.minSpeed = 30}, false); // turn to corner
  this->setClawState(Auton_Functions::OPEN);// open claw
  drive.moveToPoint(-57.23, -109.03, 2000, {.forwards = false, .minSpeed = 30}, false); // push into corner
  intake.brake(); // stop intake

  // Get fourth goal
  delay(200);
  drive.turnToHeading(84.98, 500, {}, false);
  intake.move(-127);
  drive.moveToPoint(-8.19, -99.81, 800, {.minSpeed = 15}, false); // push with intake
  drive.moveToPoint(17.94, -108.15, 1750, {.minSpeed = 30}, false);
  drive.moveToPoint(53.79, -120, 700, {.minSpeed = 30}, false);
  delay(150);
  /*chassis.moveToPoint(32.68, -120.39, 500, {.forwards = false}, false);
  chassis.turnToHeading(51.13, 500);
  stage = 1;
  intake.move(-127);
  chassis.moveToPoint(46.98, -108.68, 500, {}, false);
  chassis.moveToPoint(2.73, -110.88, 1000, {.forwards = false}, false);*/
}


//! RED FUNCTIONS

void Auton_Functions::RED_Auton::goalRush(){
    //TODO: GET GOAL
    parent.drive.setBrakeMode(E_MOTOR_BRAKE_HOLD); // Set the brake mode to hold
    parent.drive.moveToPoint(0, 24, 1000, {.minSpeed = 30});
    firstStageIntake.move(-127);
    parent.drive.moveToPoint(0, 33, 750, {.minSpeed = 40});
    parent.drive.waitUntil(3);
    parent.doinker.extend();
    parent.drive.moveToPoint(-0.2, 16.79, 1000, {.forwards = false, .maxSpeed = 127});
    parent.drive.waitUntilDone();
    parent.drive.setBrakeMode(E_MOTOR_BRAKE_COAST);
    parent.doinker.retract();
    firstStageIntake.brake();
    parent.setClawState(Auton_Functions::AUTO);
    delay(400);

    //TODO: GET GOAL #2 AND SCORE RING
    parent.drive.turnToHeading(136, 850, {.direction = AngularDirection::CW_CLOCKWISE}); // Turn to goal // 192 - 1250
    parent.drive.moveToPoint(-15.7, 25.2, 1000, {.forwards = false, .minSpeed = 30});
    parent.drive.waitUntilDone();
    parent.intake.move(-127);

    parent.drive.turnToHeading(165, 1000, {.direction = AngularDirection::CW_CLOCKWISE});
    parent.drive.waitUntilDone();

    //TODO: GET CORNER RING
    parent.drive.setBrakeMode(E_MOTOR_BRAKE_HOLD);
    delay(200);
    parent.intake.move(-63.5);
    parent.setClawState(Auton_Functions::OPEN);
    parent.intake.set_current_limit(3000, 1);
    parent.drive.moveToPoint(4.4, -16.6, 4500, {}, false);
    parent.drive.moveToPoint(6.35, -19.33, 2000, {.minSpeed = 100}, false);
    parent.drive.moveToPoint(4.66, -14.56, 1000, {.forwards = false}, false);
    parent.intake.brake();
    firstStageIntake.move(-127);
    parent.drive.moveToPoint(8.02, -22.2, 2000, {.minSpeed = 127}, false);
    parent.drive.waitUntilDone();

    // TODO: GRAB GOAL
    parent.drive.moveToPoint(-0.87, -3.17, 3000, {.forwards = false, .maxSpeed = 30}, false);
    parent.drive.waitUntilDone();
    parent.drive.turnToHeading(204, 1100, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 30});
    parent.setClawState(Auton_Functions::AUTO);
    parent.drive.moveToPose(7.75, 28.85, 204, 875, {.forwards = false, .minSpeed = 50}, false);
    parent.drive.waitUntilDone();
    parent.intake.move(-127);


    /*//TODO: GET CORNER RING
    parent.drive.setBrakeMode(E_MOTOR_BRAKE_HOLD);
    delay(200);
    parent.intake.move(-63.5);
    parent.setClawState(Auton_Functions::OPEN);
    parent.intake.set_current_limit(3000, 1);
    parent.drive.moveToPoint(4.4, -16.6, 4500, {}, false);
    parent.drive.moveToPoint(6.35, -19.33, 2000, {.minSpeed = 100}, false);
    parent.drive.moveToPoint(4.66, -14.56, 1000, {.forwards = false}, false);
    parent.drive.moveToPoint(8.02, -22.2, 2000, {.minSpeed = 127}, false);
    parent.drive.waitUntilDone();
    delay(2500);
    parent.intake.brake();
    firstStageIntake.move(-127);
    parent.drive.moveToPoint(-0.87, -3.17, 3000, {.forwards = false, .maxSpeed = 30}, false);
    parent.drive.waitUntilDone();
    parent.drive.turnToHeading(204, 1100, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 30});
    parent.setClawState(Auton_Functions::AUTO);
    parent.drive.moveToPose(7.75, 28.85, 204, 875, {.forwards = false, .minSpeed = 50});*/
}

void Auton_Functions::RED_Auton::ringRush(){

}


//? BLUE FUNCTIONS

void Auton_Functions::BLUE_Auton::goalRush(){
    //? BLUE GOAL RUSH
    parent.drive.setBrakeMode(E_MOTOR_BRAKE_HOLD);
    parent.drive.moveToPose(0.37, 31.13, 5.4, 1000, {.lead = 0.2, .minSpeed = 100});
    parent.drive.waitUntilDone();
    parent.doinker.extend();
    delay(300);
    parent.drive.moveToPoint(0.44, 18.74, 500, {.forwards = false, .maxSpeed = 127, .minSpeed = 50});
    parent.drive.waitUntilDone();
    parent.doinker.retract();
    delay(300);
    parent.drive.turnToHeading(40, 300, {.minSpeed = 56});
    firstStageIntake.move(-127);
    parent.drive.moveToPoint(9.4, 23.4, 700, {.minSpeed = 80});
    
    // Grab Goal and deposit ring

    parent.drive.waitUntilDone();
    delay(200);
    parent.drive.turnToHeading(164, 500, {.direction = AngularDirection::CW_CLOCKWISE,.minSpeed = 60});
    parent.drive.waitUntilDone();
    delay(300);
    parent.setClawState(Auton_Functions::AUTO);
    parent.drive.setBrakeMode(E_MOTOR_BRAKE_COAST);
    parent.drive.moveToPoint(8.4, 26, 600, {.forwards = false, .minSpeed = 63.5});
    parent.drive.waitUntilDone();
    firstStageIntake.brake();
    delay(150);
    parent.drive.setBrakeMode(E_MOTOR_BRAKE_HOLD);
    parent.intake.move_relative(-900, -127);
    parent.drive.moveToPoint(15.65, 2.09, 750);
    parent.drive.waitUntilDone();
    parent.drive.turnToHeading(50, 500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 50});
    parent.drive.waitUntilDone();
    delay(400);
    parent.setClawState(Auton_Functions::OPEN);

    // Get and score second goal

    delay(300);
    parent.drive.turnToHeading(76.5, 100, {.maxSpeed = 80, .minSpeed = 50});
    parent.drive.waitUntilDone();
    parent.drive.moveToPose(55.6, 19, 82.8, 2000, {.lead = 0.4, .minSpeed = 70});
    parent.drive.waitUntil(50);
    parent.intakeRaiser.extend();
    firstStageIntake.move(-127);
    parent.drive.moveToPoint(61.7, 18.4, 750);
    parent.drive.waitUntil(6);
    parent.intakeRaiser.retract();
    delay(500);
    parent.drive.turnToHeading(128.1, 400, {.minSpeed = 30});
    parent.drive.waitUntilDone();
    delay(100);
    parent.setClawState(Auton_Functions::AUTO);
    parent.drive.setBrakeMode(E_MOTOR_BRAKE_COAST);
    parent.drive.moveToPoint(38.7, 28.7, 1250, {.forwards = false, .maxSpeed = 50});
    parent.drive.waitUntilDone();
    secondStageIntake.move(-127);
    parent.drive.moveToPose(13.8, 34.3, 87.8, 2000, {.forwards = false, .maxSpeed = 45});
}

void Auton_Functions::BLUE_Auton::ringRush(){

}



// Make a PID object to use to output a voltage to the motor
lemlib::PID PID_LB(0.21, 0, 0, 16, false);
uint8_t stage = 0; // The stage of the motion that LB is in (0 = at rest | 1 = ready for ring | 2 = in the air)
double avgPosition = 0.0; // Initialized variable that holds the avgPosition of the LadyBrown rotation sensors
double targetAngle = 0.0; // Initialized variable that holds the desired position that the LadyBrown should be at

// Function to execute the Lady Brown routine in an infinite loop
void ladyBrownRoutine() {
    
    while (true) {
        // Determine the target angle based on the current stage 
        switch (stage) {
            case 1:
                targetAngle = 24; // Set target angle for stage 1
                break;
            case 2:
                targetAngle = 190; // Set target angle for stage 2
                break;
            case 4:
                targetAngle = 92; // Set target angle for hang
                break;
            default:
                stage = 0; // Reset stage to initial
                targetAngle = 0; // Set default target angle
                break;
        }

        // Calculate the average position from two rotational sensors
        avgPosition = (ladyLeftRot.get_position() * 0.01 + ladyRightRot.get_position() * 0.01) / 2;

        // Move the Lady Brown mechanism using a PID controller
        // The movement command is scaled by 12.0
        ladyBrown.move(PID_LB.update(targetAngle - avgPosition) * 12.0);

        // Delay to control the loop execution frequency
        delay(25);
    }
}