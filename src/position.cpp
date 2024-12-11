#include "position.hpp"

// distance away from the origin (in inches)
double startX = -1;
double startY = -1;
bool isRight = false;

pros::adi::Ultrasonic sonarLeft({5, 'G', 'H'}); 
pros::adi::Ultrasonic sonarBack({5, 'E', 'F'}); 
pros::adi::Ultrasonic sonarRight({5, 'A', 'B'}); 

void positionRobot(char autonID, bool value){    

  isRight = value;

  switch (autonID){
    
    case '1': // RED SKILLS
        startX = 0;
        startY = 0;
        isRight = false;
      break;

    case 'A': // BLUE SKILLS 
        startX = 0;
        startY = 0;
        isRight = false;
      break;

    case '2': // RED AWP 1
        startX = 0;
        startY = 0;
        isRight = false;
      break;

    case 'B': // BLUE AWP 1
        startX = 44.4;
        startY = 0;
        isRight = false;
      break;

    case '3': // RED AWP 2 
        startX = 0;
        startY = 0;
        isRight = false;
      break;

    case 'C': // BLUE AWP 2 
        startX = 0;
        startY = 0;
        isRight = false;
      break;

    case '4': // RED Goal Rush 
        startX = 0;
        startY = 0;
        isRight = false;
      break;

    case 'D': // BLUE Goal Rush 
        startX = 0;
        startY = 0;
        isRight = false;
      break;

    case '5': // RED Ring Rush
        startX = 0;
        startY = 0;
        isRight = false;
      break;

    case 'E': // BLUE Ring Rush
        startX = 0;
        startY = 0;
        isRight = false;
      break;
    
  }  

  if( !(startX+startY < 0) ){
    pros::screen::print(pros::E_TEXT_MEDIUM, 263, 75, "X, Y, and Angle = 0");
    pros::screen::print(pros::E_TEXT_LARGE, 286, 110, "X: %.2f", startX - getXDisplacement() );
    pros::screen::print(pros::E_TEXT_LARGE,286, 145, "Y: %.2f", startY - getYDisplacement() );
    pros::screen::print(pros::E_TEXT_MEDIUM, 265, 175, "Is Right?: %d", isRight);
    pros::screen::print(pros::E_TEXT_SMALL, 263, 200, "Bot: (%.2f, %.2f)", cmToInches(getXDisplacement()), cmToInches(getYDisplacement()));
  }
}

double getXDisplacement(){
  return isRight ? 144-(cmToInches(sonarRight.get_value())+7.85875) // If is right
                 : cmToInches(sonarLeft.get_value())+7.73875; // else
}

double getYDisplacement(){
  return cmToInches(sonarBack.get_value()) + 5.7725;
}

double cmToInches(double cm){ return cm / 2.54; }