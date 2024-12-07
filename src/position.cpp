#include "position.hpp"

// distance away from the origin (in inches)
double startX;
double startY;
bool isRight = false;

void positionRobot(short autoNum){    

  switch (autoNum){
    case 1:
        startX = 0;
        startY = 0;
        isRight = true; // Depending on skills
      break;

    case 2: // AWP 1
        startX = 47.66;
        startY = 11;
        isRight = false; // left
      break;

    case 3: // AWP 2 
        startX = 47.66;
        startY = 11;
        isRight = false; // left
      break;

    case 4: // Goal Rush 
        startX = 118.5;
        startY = 11.8;
        isRight = true; // right
      break;

    case 5: // Ring Rush
        startX = 98.2;
        startY = 14;
        isRight = true; // left
      break;
  }  

  if(autoNum != 0){
    
    /*if (autoNum != 5){
      Brain.Screen.printAt(263, 200, false, "Bot: (%.2f, %.2f)", cmToInches(getXDisplacement()), cmToInches(getYDisplacement()));
    } else {*/ 
      Brain.Screen.printAt(263, 200, false, "Bot: (%.2f, %s)", cmToInches(144-getXDisplacement()), "Y=18.25"); 
    //} 

    startX = inchesToCm(startX);
    startY = inchesToCm(startY);
    
    Brain.Screen.printAt(263, 75, false, "X, Y, and Angle = 0");
    Brain.Screen.setFont(mono30);
    Brain.Screen.printAt(286, 110, false, "X: %.2f", cmToInches(startX - getXDisplacement()) );
    Brain.Screen.printAt(286, 145, false, "Y: %.2f", cmToInches(startY - getYDisplacement()) );
    Brain.Screen.printAt(265, 175, false, "Is Right?: %d", isRight);
    Brain.Screen.setFont(mono20);
  }
}

double getXDisplacement(){
  return isRight ? inchesToCm(144-(sonarRight.distance(inches)+7.85875)) : inchesToCm(sonarLeft.distance(inches)+7.73875);
}

double getYDisplacement(){
  return inchesToCm(sonarBack.distance(inches) + 5.7725);
}
