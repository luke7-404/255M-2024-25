#include "vex.h" //Includes all the vex library

// Include my libraries
#include "printing.hpp"
#include "drivetrain.hpp"
#include "auton_functions.hpp"

using namespace vex;

// A global instance of competition
competition Competition;

LCD_Menu Menu; // Create a Menu object
data_File File(Brain.SDcard.isInserted()); // Create a File object
PID_Data PID_Obj; // Create a PID object
Odom_Data Odom_Obj; // Create a Odom object

// Create Tasks and set the priorities
task odometryTask(positionTracking, task::taskPriorityHigh);
task PID_Task(driveControl, task::taskPriorityHigh);

// A unsigned integer that represents the selected autonomous program
uint8_t autoNum;

// If false, stops the menu handler loop
bool MenuON = true;

// A logic variable to print only once
bool printed = false;

// A integer function that manages function calls for the Menu object
int MenuHandler(){
  while(MenuON){
    // Checks if the menu is not deconstructed
    if(!Menu.isDeconstructed){
      
      // If less than or equal to 26, we are finding the tab buttons 
      if (Brain.Screen.yPosition() <= 26){
        checkPressedTab(Brain.Screen.xPosition(), Menu, PID_Obj, Odom_Obj, File);
      }

      // If the file was emphasized and not printed, print only once
      if (File.emphasized && !printed){
        LCD.printAt(155, 148, false, "File Emphasized!");
        printed = true;
      } 

      // If the selected tab was "Auton" check if a auton button was pressed
      if(Menu.enableAuton){
        // Stores the value of the selected autonomous program in autoNum
        autoNum = checkPressedAuton(Menu, Brain.Screen.xPosition(), Brain.Screen.yPosition());
        positionRobot(autoNum);
      }
    
      LCD.render(); // Render the screen (fixes flicker)
      task::sleep(250); // Refresh the screen every quarter of a second
    }
  }
  return 1; // Return any number
}

// A boolean that determines if the data collection loop should run
bool runCollection = false;

// Gathers data from various sources and appends it to a file
int gatherData(){
  
  // Gather data from various sources
  while (runCollection){
    
    // Append data to file
    File.append_Data(Brain.timer(sec), autoNum, get_Auton_Data(PID_Obj, Odom_Obj), get_Motor_Data());
    
    task::sleep(20); // Cycle time
  }
  return 1;
}

void autonomous(void) {
  reset_Tracking_Wheels();
  // If the robot is connected to a competition field, we want to save performance
  if (Competition.isFieldControl()) {
    // Emphasize the file name if connect to competition field
    if(!File.emphasized) File.emphasizeFile(); 
    Menu.~LCD_Menu(); // destroy the Menu object
    task::stop(MenuHandler); // Stop the Menu handler task
    MenuON = false; // Ensure the loop stops
  }

  // Start the data collection task if the robot has an SD card
  if(File.isSDInserted){
    File.createFile(); // Create the CSV file and write the header

    // If the name has been created successfully,
    // Lets the data collection loop run
    if(File.isNameCreated){ 
      runCollection = true;
    }
    task dataTask(gatherData, task::taskPrioritylow); // Start task for data collection
  }

  /*turnToAngle(45);
  waitUntil(!runControl);
  turnToAngle(0);
  waitUntil(!runControl);
  turnToAngle(45);
  waitUntil(!runControl);
  turnToAngle(90);
  waitUntil(!runControl);
  turnToAngle(135);
  waitUntil(!runControl);
  turnToAngle(180);
  waitUntil(!runControl);
  turnToAngle(225);
  waitUntil(!runControl);
  turnToAngle(270);
  waitUntil(!runControl);
  turnToAngle(315);
  waitUntil(!runControl);
  turnToAngle(0);
  waitUntil(!runControl);
  turnToAngle(45);
  waitUntil(!runControl);
  turnToAngle(90);
  waitUntil(!runControl);*/
  driveToPoint(72, 72, 10000);
  //driveToPoint(48, 48, 50000);
  /*for (size_t i = 0; i < 10; i++)
  {
    turnTo(Inert.heading(degrees)+90);
    waitUntil(!runControl);
  }*/
  
  
  //turnToPoint(0, 144);
  
  // The case number is related to the value of the input variable
  switch (autoNum){
    case 0:
      std::cout << "None Auton" << std::endl;
      break;
    case 1:
      std::cout << "Skills" << std::endl;
      break;
    case 2:
      std::cout << "AWP1" << std::endl; // left
      break;
    case 3:
      std::cout << "AWP2" << std::endl; // right
      break;
    case 4:
      std::cout << "Goal Rush" << std::endl; // right
      break;
    case 5:
      std::cout << "Ring Rush" << std::endl; // left
      break;
  }
}

// This is the main execution loop for the user control program.
  // Each time through the loop your program should update motor


void usercontrol(void) {
    
  while (1) {
    runControl = false;
    joeySticks(Ctrl.Axis3.value(), Ctrl.Axis1.value());


    // if the intake is stuck and either bumper button is being pressed get intake unstuck
    if(isIntakeStuck() && (Ctrl.ButtonL1.pressing() || Ctrl.ButtonL2.pressing())) {
      intake.spinFor(fwd, 360, deg, 100, velocityUnits::pct);
      
    } else if(intake.isDone()) { // if the intake is done spinning to get unstuck then enable control

      if (Ctrl.ButtonL1.pressing()){                // if the top bumper is being held down  
        intake.spin(fwd, 100, pct);                 // out-take
      } else if (Ctrl.ButtonL2.pressing()){         // if the bottom bumper is being held down
        intake.spin(directionType::rev, 100, pct);  // intake
      } else intake.stop(coast);                    // if all else, stop
    }

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

// Function to handle the GUI interaction on the file screen
void fileGUIHandle(){
  if(Menu.enableFile && File.isSDInserted){
    checkPressedFile(File, Brain.Screen.xPosition(), Brain.Screen.yPosition()); 
  }
}

void pistonGo(){
  piz.set(!piz.value());
}

task menuTab(MenuHandler, task::taskPrioritylow);

// Main will set up the competition functions and callbacks.
int main() {

  // Resets sensors
  calibrate_IMU();
  reset_Tracking_Wheels();

  Ctrl.ButtonA.pressed(pistonGo);

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // When the brain screen is released, run the fileGUIHandle function
  Brain.Screen.released(fileGUIHandle); 

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}