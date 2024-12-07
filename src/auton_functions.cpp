#include "auton_functions.hpp"

competition isAutonEvent;
Odom_Data odom;
int8_t release = 1;
int autoClampClaw(){
    while (isAutonEvent.isAutonomous()) {
        
             if(release == 1) { mogoClaw.set(false); }
        else if(release == -1) { mogoClaw.set(true); }
        else if(clawDistance.isObjectDetected() && (clawDistance.objectDistance(inches) > 6.0 && clawDistance.objectDistance(inches) < 7.4)){
            mogoClaw.set(true);
        } else mogoClaw.set(false);

        task::sleep(75);
    }
    return 1;
}

void Skills(){
    release = 0;
    driveToPoint(72, -2, 0, 0, 1000);
    waitUntil(!runControl);
    turnToAngle(60, 2000);
    intake.spin(reverse, 12, volt);
    waitUntil(!runControl);
    driveToPoint(50, -2, 60, 2);
    waitUntil(!runControl);
    intake.stop(coast);
    turnToAngle(240);

}


void AWP1(){
  release = -1;
  turnToAngle(270, 1000);
  waitUntil(!runControl);
  driveToPoint(72, 8.75, 0, 1, 4000);
  waitUntil(getYDisplacement() < 10.5);
  wait(0.5, sec);
  intake.spin(reverse, 12, volt);
  waitUntil(!runControl);
  odom.setPosition(72, 9.55);
  wait(2, sec);
  driveToPoint(72, 19, 0);
  waitUntil(!runControl);
  release = 0;
  wait(0.5, sec);
  turnToAngle(135, 800);
  waitUntil(!runControl);
  driveToPoint(48, 48, 135, 1);
}

void goalRush(){ 
  driveToPoint(118.5, 50, 0, 2, 1500);
  intake.spin(reverse, 100, pct);
  release = 0;
  waitUntil(!runControl);
  doinker.set(true);
  turnToAngle(270, 1000);
  waitUntil(!runControl);
  turnToAngle(0);

}


void AWP2(){}


void ringRush(){ 
  //wait(4.25, sec);
    release = -1;
    turnToAngle(90);
    waitUntil(!runControl);
    driveToPoint(46.5, 9.5, 340, true);
    waitUntil(!runControl);
    odom.setPosition(47.5, 9.5);
    intake.spin(reverse, 12, volt);
    wait(1.75, sec);
    intakeSecond.spinFor(fwd, 35.0, rotationUnits::deg, 20, velocityUnits::dps);
    release = 0;
    intakeRaiser.set(true);
    driveToPoint(47.5, 24, 0);
    intake.spin(reverse, 5.9, volt);
    wait(0.2, sec);
    intakeRaiser.set(false);
    /*wait(0.6, sec);
    intake.stop(coast);*/
    /*turnToAngle(182.5);
    waitUntil(!runControl);
    driveToPoint(53, 30, 184, true, 400);
    waitUntil(mogoClaw.value() == true);
    //intake.spin(reverse, 12, volt);
    wait(2, sec);
    turnToAngle(225);
    waitUntil(!runControl);
    driveToPoint(30, 48, 225, true, 650);
    moveLadyBrown(24, true);
    stage = 2;*/
}


/**
 * @brief Ring Rush
 * 
 * Starting point(72, 9.5)
    


*/