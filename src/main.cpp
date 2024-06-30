#include "vex.h" //Includes all the vex library

// Include my libraries
#include "printing.hpp"

using namespace vex;

// A global instance of competition
competition Competition;

// Create a global Menu Object
LCD_Menu Menu;
data_collect Data(Brain.SDcard.isInserted());


// A unsigned integer that represents the selected autonomous program
uint8_t autoNum;

// If false, stops the menu handler loop
bool MenuON = true;

// A integer function that manages function calls for the Menu object
int MenuHandler(){
  while(MenuON){
    // If less than or equal to 26, we are finding the TAB buttons 
    if (Brain.Screen.yPosition() <= 26){
      if (Brain.Screen.xPosition() > 384) {
        Menu.printFile(Data); // Prints File/ graph information page
        Menu.enableFile = true; // Toggles file buttons functionality to on
        Menu.enableAuton = false; // Toggles auton buttons functionality to off
      } else checkPressedTab(Menu); // Calls the pressed tab function
    }
    
    // If the selected tab was "Auton" check if a auton button was pressed
    if(Menu.enableAuton){

      // Stores the value of the selected autonomous program in autoNum
      autoNum = checkPressedAuton(Menu, Brain.Screen.xPosition(), Brain.Screen.yPosition());
    }
    LCD.render();

    wait(250, msec); // Refresh the screen every quarter of a second
  }
  return 1; // Return any number
}


void pre_auton(void) {
  vexcodeInit();

}


void autonomous(void) {

  // If the robot is connected to a competition field, we want to save performance
  if (Competition.isFieldControl() || Competition.isCompetitionSwitch()) {
    Menu.~LCD_Menu(); // destroy the Menu object
    task::stop(MenuHandler); // Stop the Menu handler task
    MenuON = false; // Ensure the loop stops
  }

  if(Data.isSDInserted){
    if(!Data.emphasized) Data.emphasizeFile();
    Data.createFile();
  }

  /*
    Test if the output of the checkAuton function outputs the correct value

    The case number is related to the value of the input variable
  */
  switch (autoNum){
    case 0:
      std::cout << "None Auton" << std::endl;
      break;
    case 1:
      std::cout << "Skills" << std::endl;
      break;
    case 2:
      std::cout << "AWP1" << std::endl;
      break;
    case 3:
      std::cout << "AWP2" << std::endl;
      break;
    case 4:
      std::cout << "Goal Rush" << std::endl;
      break;
    case 5:
      std::cout << "Ring Rush" << std::endl;
      break;
  }

}


void usercontrol(void) {
  
  // This is the main execution loop for the user control program.
  // Each time through the loop your program should update motor
  while (1) {
    
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

void fileGUIHandle(){
  if(Menu.enableFile && Data.isSDInserted){
    checkPressedFile(Data, Brain.Screen.xPosition(), Brain.Screen.yPosition()); 
  }
}

// Main will set up the competition functions and callbacks.
int main() {

  // Print the first screen
  Menu.printAuton();
  
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();
  

  Brain.Screen.released(fileGUIHandle);

  // Create a task for the menu handler function and set the priority low
  task menuTab(MenuHandler, task::taskPrioritylow);

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}