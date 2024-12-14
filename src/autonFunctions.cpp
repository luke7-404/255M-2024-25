#include "autonFunctions.hpp"


std::int8_t Auton_Functions::release = 1;
void Auton_Functions::autoClampClaw(){
    while (competition::is_autonomous()) {
        
             if(release == 1) { mogoClaw.retract(); }
        else if(release == -1) { mogoClaw.extend(); }
        else if(clawDistance.get_distance() > 140 && clawDistance.get_distance() < 170){
            mogoClaw.extend();
        } else mogoClaw.retract();

        delay(50);
    }
}

Auton_Functions::Auton_Functions(lemlib::Chassis& drive, ADIPneumatics& intakeRaiser, 
                                 ADIPneumatics& mogoClaw, ADIPneumatics& doinker, MotorGroup& intake)
        : drive(drive), intakeRaiser(intakeRaiser), mogoClaw(mogoClaw), doinker(doinker), intake(intake)
{
    std::cout << "Auton Func obj created and chassis obj passed" << std::endl;
}


//! RED FUNCTIONS

void Auton_Functions::RED_Auton::Skills(){
  
}

void Auton_Functions::RED_Auton::AWP1(){
  parent.drive.swingToHeading(90, DriveSide::LEFT, 1000, {.direction = AngularDirection::CW_CLOCKWISE});
  parent.drive.waitUntilDone();
  parent.intake.move(-127);
  delay(500);
  parent.intake.brake();
  parent.drive.moveToPose(11.0, 14, 58.25, 2500);
  parent.drive.waitUntilDone();
  parent.drive.turnToHeading(233.5, 2500);
  parent.drive.waitUntilDone();
  parent.release = 0;
  parent.drive.moveToPoint(21, 24, 1250, {.forwards = false, .minSpeed = -31.75});
  parent.drive.waitUntilDone();
  parent.drive.turnToHeading(326, 2250, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
  parent.drive.waitUntilDone();
  parent.intake.move_voltage(-12000);
  parent.drive.turnToHeading(355.45, 1500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
  parent.drive.waitUntilDone();
  parent.drive.moveToPoint(18, 33.2, 2000, {.forwards = true, .minSpeed = 0});
  parent.drive.waitUntilDone();
  parent.drive.moveToPose(18, 2, 180, 2000, {.forwards = true, .maxSpeed = 63.5, .minSpeed = -63.5});
  parent.drive.waitUntilDone();
  parent.intake.brake();
}

void Auton_Functions::RED_Auton::AWP2(){
  /*MAKE SURE TO CUT AND PASTE
    1. copy and paste lines
    2. control shift I 
    3. fn + left arrow
    4. add parent.
    5. add to blue wp2
    */

  parent.drive.swingToHeading(260, DriveSide::RIGHT, 2000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
  parent.drive.waitUntilDone();
  parent.intake.move(-127);
  delay(1500);
  parent.drive.moveToPose(-16.54, 8.52, -67.88, 4000);
  parent.drive.waitUntilDone();
  parent.drive.turnToHeading(-217.38, 2000);
  parent.drive.waitUntilDone();
  parent.drive.moveToPoint(-33.65, 19.59, 3000, {.forwards = false, .minSpeed = -63.5});
  parent.drive.waitUntilDone();
  parent.drive.moveToPose(-31.8, 20.4, -225.22, 3000);
  parent.drive.waitUntilDone();
  parent.drive.turnToHeading(-350.16, 2000);
  parent.drive.waitUntilDone();
  parent.drive.moveToPoint(-26.6, 28.54, 2000, {.forwards = true, .minSpeed = 0});
  parent.drive.waitUntilDone();
  parent.drive.turnToHeading(-170.16, 2000);
  parent.drive.waitUntilDone();
  parent.drive.moveToPoint(-29.23, 13.95, 2000, {.forwards = true, .minSpeed = 0});
}

void Auton_Functions::RED_Auton::goalRush(){
  
}

void Auton_Functions::RED_Auton::ringRush(){

}


//? BLUE FUNCTIONS

void Auton_Functions::BLUE_Auton::Skills(){
  
}

void Auton_Functions::BLUE_Auton::AWP1(){
  parent.drive.swingToHeading(90, DriveSide::LEFT, 1000, {.direction = AngularDirection::CW_CLOCKWISE});
  parent.drive.waitUntilDone();
  parent.intake.move(-127);
  delay(500);
  parent.intake.brake();
  parent.drive.moveToPose(11.0, 14, 58.25, 2500);
  parent.drive.waitUntilDone();
  parent.drive.turnToHeading(233.5, 2500);
  parent.drive.waitUntilDone();
  parent.release = 0;
  parent.drive.moveToPoint(21, 24, 1250, {.forwards = false, .minSpeed = -31.75});
  parent.drive.waitUntilDone();
  parent.drive.turnToHeading(326, 2250, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
  parent.drive.waitUntilDone();
  parent.intake.move_voltage(-12000);
  parent.drive.turnToHeading(355.45, 1500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
  parent.drive.waitUntilDone();
  parent.drive.moveToPoint(18, 33.2, 2000, {.forwards = true, .minSpeed = 0});
  parent.drive.waitUntilDone();
  parent.drive.moveToPose(18, 2, 180, 2000, {.forwards = true, .maxSpeed = 63.5, .minSpeed = -63.5});
  parent.drive.waitUntilDone();
  parent.intake.brake();
}

void Auton_Functions::BLUE_Auton::AWP2(){

  parent.drive.swingToHeading(260, DriveSide::RIGHT, 2000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
  parent.drive.waitUntilDone();
  parent.intake.move(-127);
  delay(1500);
  parent.drive.moveToPose(-16.54, 8.52, -67.88, 4000);
  parent.drive.waitUntilDone();
  parent.drive.turnToHeading(-217.38, 2000);
  parent.drive.waitUntilDone();
  parent.drive.moveToPoint(-33.65, 19.59, 3000, {.forwards = false, .minSpeed = -63.5});
  parent.drive.waitUntilDone();
  parent.drive.moveToPose(-31.8, 20.4, -225.22, 3000);
  parent.drive.waitUntilDone();
  parent.drive.turnToHeading(-350.16, 2000);
  parent.drive.waitUntilDone();
  parent.drive.moveToPoint(-26.6, 28.54, 2000, {.forwards = true, .minSpeed = 0});
  parent.drive.waitUntilDone();
  parent.drive.turnToHeading(-170.16, 2000);
  parent.drive.waitUntilDone();
  parent.drive.moveToPoint(-29.23, 13.95, 2000, {.forwards = true, .minSpeed = 0});
}

void Auton_Functions::BLUE_Auton::goalRush(){
  
}

void Auton_Functions::BLUE_Auton::ringRush(){

}
