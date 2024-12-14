#include "position.hpp"

// X distance away from the origin (bottom left corner of field) in inches
float startX;

// Y distance away from the origin (bottom left corner of field) in inches
float startY;

// Logic boolean to tell if the robot should read from the left (false) or the right (true)
bool isRight = false;


pros::adi::Ultrasonic sonarLeft({5, 'G', 'H'}); // distance sensor on the left
pros::adi::Ultrasonic sonarBack({5, 'E', 'F'}); // distance sensor in the back
pros::adi::Ultrasonic sonarRight({5, 'A', 'B'}); // distance sensor on the right

// Function to position the robot based on the autonomous ID
void positionRobot(char autonID){    

  // Switch conditional statement to choose from any of the 10 scenarios
  switch (autonID){
    
    case '1': // RED SKILLS
        startX = 0; // Our desired starting X position (inches)
        startY = 0; // Our desired starting Y position (inches)
        isRight = false; // Are we starting on the left (False) or right (True) side of the field
      break; // END CASE

    case 'A': // BLUE SKILLS 
        startX = 0; // Our desired starting X position (inches)
        startY = 0; // Our desired starting Y position (inches)
        isRight = false; // Are we starting on the left (False) or right (True) side of the field
      break; // END CASE

    case '2': // RED AWP 1
        startX = 12.8; // Our desired starting X position (inches)
        startY = 0; // Our desired starting Y position (inches)
        isRight = false; // Are we starting on the left (False) or right (True) side of the field
      break; // END CASE

    case 'B': // BLUE AWP 1
        startX = 12.8; // Our desired starting X position (inches)
        startY = 0; // Our desired starting Y position (inches)
        isRight = false; // Are we starting on the left (False) or right (True) side of the field
      break; // END CASE

    case '3': // RED AWP 2 
        startX = 12.0; // Our desired starting X position (inches)
        startY = 0; // Our desired starting Y position (inches)
        isRight = true; // Are we starting on the left (False) or right (True) side of the field
      break; // END CASE

    case 'C': // BLUE AWP 2 
        startX = 12.0; // Our desired starting X position (inches)
        startY = 0; // Our desired starting Y position (inches)
        isRight = true; // Are we starting on the left (False) or right (True) side of the field
      break; // END CASE

    case '4': // RED Goal Rush 
        startX = 0; // Our desired starting X position (inches)
        startY = 0; // Our desired starting Y position (inches)
        isRight = false; // Are we starting on the left (False) or right (True) side of the field
      break; // END CASE

    case 'D': // BLUE Goal Rush 
        startX = 0; // Our desired starting X position (inches)
        startY = 0; // Our desired starting Y position (inches)
        isRight = false; // Are we starting on the left (False) or right (True) side of the field
      break; // END CASE

    case '5': // RED Ring Rush
        startX = 0; // Our desired starting X position (inches)
        startY = 0; // Our desired starting Y position (inches)
        isRight = false; // Are we starting on the left (False) or right (True) side of the field
      break; // END CASE

    case 'E': // BLUE Ring Rush
        startX = 0; // Our desired starting X position (inches)
        startY = 0; // Our desired starting Y position (inches)
        isRight = false; // Are we starting on the left (False) or right (True) side of the field
      break; // END CASE
    
  }  

  // Check if the autonID is within the valid ASCII range for printing
  if( (autonID < 70 && autonID > 64) || (autonID < 54 && autonID > 48) ){
    // Print the robot's orientation and position on the screen
    pros::screen::print(pros::E_TEXT_MEDIUM, 295, 70, "Is Right?: %d", isRight);
    pros::screen::print(pros::E_TEXT_LARGE,  278, 100, "X:%.1f", startX - getXDisplacement() );
    pros::screen::print(pros::E_TEXT_LARGE, 278, 140, "Y:%.1f", startY - getYDisplacement() );
    pros::screen::print(pros::E_TEXT_MEDIUM, 272, 180, "Bot: (%.1f, %.1f)", getXDisplacement(), getYDisplacement());
  }
}

// Function to calculate the X displacement of the robot
float getXDisplacement() {
  return (isRight ? cmToInches(sonarRight.get_value() / 10) // Use right sensor if isRight
                  : cmToInches(sonarLeft.get_value() / 10) // Use left sensor otherwise
          ) + 7.25; // Add to the selected value offset
}

// Function to calculate the Y displacement of the robot
float getYDisplacement() {
  return cmToInches(sonarBack.get_value() / 10) + 7.5; // Convert cm to inches and add offset
}

// Utility function to convert centimeters to inches
float cmToInches(float cm) {
  return (cm / 2.54); // Conversion factor from cm to inches
}