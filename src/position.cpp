#include "position.hpp"

// distance away from the origin (in inches)
double startX;
double startY;
double moveToX;
double moveToY;
double sensorX;
double sensorY;
double x_distance;
bool isRight = false;

void positionRobot(uint8_t autoNum){    
    switch (autoNum){
        case 1:
            startX = 0;
            startY = 0;
            isRight = true; // Depending on skills
            break;

        case 2:
            startX = 0;
            startY = 0;
            isRight = false;
            break;

        case 3:
            startX = 0;
            startY = 0;
            isRight = true;
            break;

        case 4:
            startX = 0;
            startY = 0;
            isRight = true;
            break;

        case 5:
            startX = 0;
            startY = 0;
            isRight = false;
            break;
    }

    if (isRight){
        sensorX = sonarRight.distance(inches);
    } else sensorX = sonarLeft.distance(inches);
    
    sensorY = sonarBack.distance(inches);

    moveToX = startX - sensorX;
    
    if (isRight){
        x_distance = 144 - sensorX;
    } else x_distance = sensorX;
    

    moveToY = startY - sensorY;
    
    if(autoNum > 0){
        Brain.Screen.printAt(263, 75, false, "X, Y, and Angle = 0");
        Brain.Screen.setFont(mono30);
        Brain.Screen.printAt(286, 110, false, "X: %.2f", moveToX);
        Brain.Screen.printAt(286, 145, false, "Y: %.2f", moveToY);
        Brain.Screen.printAt(265, 175, false, "Is Right?: %d", isRight);
        Brain.Screen.setFont(mono20);
    }
}

/*

int position(void){
  
  // Desired position values
  uint8_t xPos = 19; // inches away horizontally
  uint8_t yPos = 7; // 7 inches away vertically
  uint16_t inertDeg = 350;

  // Use the distance sensors, sideX and sideY, to get the distance to the  
  // perimeters in inches and store them in inputX and inputY variables
  uint8_t inputX = rightSide.objectDistance(inches); 
  uint8_t inputY = rear.objectDistance(inches);
  short inert = Inertial.heading(deg);

  // Definitions of the integer variables diffX and diffY
  // These will hold the difference of a position and the input
  // side, respectively
  short diffX, diffY;

  // While the boolean positionOn is true, 
  // it will loop the code in the body
  while (positionOn){
    

    // for our auton 1 and skills we face a different direction
    // if runAuton does not equal 2 then xPos, yPos, and inertDeg
    // equal different values
    if(runAuton != 2){
      yPos = 6; // 6 inches for yPos
      inertDeg = 31; // 31 degrees to the right
      xPos = 13; // 13 inches for xPos
    } else { 
      // if else, then xPos, yPos, and inertDeg are back to their default
      xPos = 19;
      yPos = 7;
      inertDeg = 350;
    }

    // If the value is not than 0, then it calculates the 
    // difference for that value, otherwise it does not
    if (xPos != 0) {
      Brain.Screen.clearLine(2);
      // Calculate and indicate how far way the position is
      diffX = xPos - inputX;
      Brain.Screen.printAt(6, 40, "Move bot %d horizontally", diffX);
    } 

    // If the value is not than 0, then it calculates the 
    // difference for that value, otherwise it does not
    if (yPos != 0){
      Brain.Screen.clearLine(3);

      // Calculate and indicate how far way the position is
      diffY = yPos - inputY; 
      Brain.Screen.printAt(6, 60, "Move bot %d vertically", diffY);
    }

    if (inertDeg != 0){
      Brain.Screen.clearLine(8);
      Brain.Screen.printAt(0, 160, "Inertial Position %d", inert - inertDeg);
    }
      
    // Update sensor variables
    inputX = rightSide.objectDistance(inches); 
    inputY = rear.objectDistance(inches);
    inert = Inertial.heading(deg);

    task::sleep(50); // save CPU resources
  }
  return 1;
}


*/