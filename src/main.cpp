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

// A short integer that represents the selected autonomous program
short autonSelected = 0;

// If false, stops the menu handler loop
bool MenuON = true;

// A logic variable to print only once
bool printed = false;


// A integer function that manages function calls for the Menu object
int MenuHandler(){
  checkPressedAuton(Menu, 30, 60);
  while(MenuON){
    // Checks if the menu is not deconstructed
    if(!Menu.isDeconstructed){
      Brain.Screen.render(true); // Render the screen (fixes flicker)

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
        // Stores the value of the selected autonomous program in autonSelected
        autonSelected = checkPressedAuton(Menu, Brain.Screen.xPosition(), Brain.Screen.yPosition());
        positionRobot(autonSelected);
      }
    
      Brain.Screen.render(true); // Render the screen (fixes flicker)
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
    // When the robot is not disabled and the robot is active
    if (Competition.isAutonomous() || Competition.isDriverControl()){
      // Append data to file
      File.append_Data(Brain.timer(sec), autonSelected, get_Auton_Data(PID_Obj, Odom_Obj), get_Motor_Data());
    }
    task::sleep(20); // Cycle time
  }
  return 1;
}

void autonomous(void) {
  reset_Tracking_Wheels();
  // If the robot is connected to a competition field, we want to save performance
  if (Competition.isFieldControl()) {
    Menu.~LCD_Menu(); // destroy the Menu object
    task::stop(MenuHandler); // Stop the Menu handler task
    MenuON = false; // Ensure the loop stops
  }

  task autoClamp(autoClampClaw, task::taskPrioritylow);

  double x = cmToInches(getXDisplacement());
  double y = cmToInches(getYDisplacement());

  // The case number is related to the value of the input variable
  switch (autonSelected){
    case 0:
      std::cout << "None Auton" << std::endl;
      break;
    case 1:
      std::cout << "Skills" << std::endl;
      Odom_Obj.setPosition(72,11.5);
      Skills();
      break;
    case 2:
      std::cout << "AWP1" << std::endl; // left
      Odom_Obj.setPosition(x, y);
      AWP1();
      break;
    case 3:
      std::cout << "AWP2" << std::endl; // left
      Odom_Obj.setPosition(x, y);
      AWP2();
      break;
    case 4:
      std::cout << "Goal Rush" << std::endl; // right
      Odom_Obj.setPosition(x, y);
      goalRush();
      break;
    case 5:
      std::cout << "Ring Rush" << std::endl; // left
      //Odom_Obj.setPosition(144-x, 18.25);
      ringRush();
      break;
    
  }
}

// This is the main execution loop for the user control program.
// Each time through the loop your program should update motor
void usercontrol(void) {
    
  while (1) {
    runControl = false;
    joeySticks(Ctrl.Axis3.value(), Ctrl.Axis1.value());

    intakeFirst.setVelocity(350, rpm);
    intakeSecond.setVelocity(100, pct);
  
    if (Ctrl.ButtonL1.pressing()){         // if the top bumper is being held down  
      intake.spin(fwd);                    // out-take
    } else if (Ctrl.ButtonL2.pressing()){  // if the bottom bumper is being held down
      intake.spin(reverse);                // intake
    } else intake.stop(coast);             // if all else, stop
    

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

/// @brief When called, sets the state of the mogoClaw to the opposite value
void toggleClaw(){ mogoClaw.set(!mogoClaw.value()); }

/// @brief When called, sets the state of the doinker to the opposite value
void toggleDoinker(){ doinker.set(!doinker.value()); }

/// @brief When called, sets the state of the intakeRaiser to the opposite value
void toggleRaiser(){ intakeRaiser.set(!intakeRaiser.value()); }


task menuTab(MenuHandler);

// Main will set up the competition functions and callbacks.
int main() {

  // Resets sensors
  reset_LadyBrown();
  calibrate_IMU();
  reset_Tracking_Wheels();

  // CALLBACKS 
  Ctrl.ButtonA.pressed(toggleClaw);
  Ctrl.ButtonB.pressed(toggleDoinker);
  Ctrl.ButtonY.pressed(toggleRaiser);
  Ctrl.ButtonR2.pressed(LadyBrownRoutine);

  // When the brain screen is released, run the fileGUIHandle function
  Brain.Screen.released(fileGUIHandle); 

  // Create Tasks and set the priorities
  task odometry_Task(positionTracking, task::taskPriorityHigh);
  task PID_Task(driveControl, task::taskPriorityHigh);
  task LadyBrown_Task(ladyBrownControl, task::taskPriorityNormal);

  // Start the data collection task if the robot has an SD card
  if(File.isSDInserted && !File.isNameCreated){

    // Emphasize the file name if connect to competition field
    if (Competition.isFieldControl() && !File.emphasized) File.emphasizeFile(); 

    File.createFile(); // Create the CSV file and write the header
    
    // If the name has been created successfully, let the data collection loop run
    if(File.isNameCreated) runCollection = true;

    task dataTask(gatherData, task::taskPrioritylow); // Start task for data collection
  }

  if(!Competition.isAutonomous()) task::stop(autoClampClaw);

  // Set up callbacks for autonomous and driver control periods.
  //Competition.test_disable();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}