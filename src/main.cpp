#include "vex.h" //Includes all the vex library

// Include my libraries
#include "printing.hpp"
#include "auton_functions.hpp"

using namespace vex;

// A global instance of competition
competition Competition;

LCD_Menu Menu; // Create a Menu object
data_File File(Brain.SDcard.isInserted()); // Create a File object
PID_Data PID; // Create a PID object
Odom_Data Odom; // Create a Odom object



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
        checkPressedTab(Brain.Screen.xPosition(), Menu, PID, Odom, File);
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
  std::vector<double> auton_data; // create vector for auton data
  //std::vector<double> motor_data; // create vector for motor data
  
  // Gather data from various sources
  while (runCollection){
    
    // PID Data
    {
      auton_data.push_back(PID.error);
      auton_data.push_back(PID.prevError);
      auton_data.push_back(PID.integral);
      auton_data.push_back(PID.derivative);
      auton_data.push_back(PID.drivePowerPID);
      // Convert turn data to degrees
      auton_data.push_back(radToDeg(PID.turn_Error));
      auton_data.push_back(radToDeg(PID.turn_PrevError));
      auton_data.push_back(radToDeg(PID.turn_Integral));
      auton_data.push_back(radToDeg(PID.turn_Derivative));
      auton_data.push_back(PID.turnPowerPID);
    }
    
    // Odom Data
    {
      // Left sensor data
      auton_data.push_back(Odom.LPos);
      auton_data.push_back(Odom.LPrevPos);
      auton_data.push_back(Odom.deltaDistL);
      // Right sensor data
      auton_data.push_back(Odom.RPos);
      auton_data.push_back(Odom.RPrevPos);
      auton_data.push_back(Odom.deltaDistR);
      // Perpendicular sensor data
      auton_data.push_back(Odom.SPos);
      auton_data.push_back(Odom.SPrevPos);
      auton_data.push_back(Odom.deltaDistS);
      // Inertial sensor data
      auton_data.push_back(Odom.currentAbsoluteOrientation);
      auton_data.push_back(Odom.prevTheta);
      auton_data.push_back(Odom.deltaTheta);
      // Arc average
      auton_data.push_back(Odom.avgThetaForArc);
      // X-Coordinate Data
      auton_data.push_back(Odom.deltaXLocal); // Change in robot relative position in the X direction
      auton_data.push_back(Odom.deltaXGlobal); // Change in field relative position in the X direction
      auton_data.push_back(Odom.xPosGlobal); // Current field relative position in the X direction
      // Y-Coordinate Data
      auton_data.push_back(Odom.deltaYLocal); // Change in robot relative position in the Y direction
      auton_data.push_back(Odom.deltaYGlobal); // Change in field relative position in the Y direction
      auton_data.push_back(Odom.yPosGlobal); // Current field relative position in the Y direction
    }
  
    // Motor Data
    /*{
      motor_data.push_back(averageTemp()); // Get the average temperature
      motor_data.push_back(averageVolt()); // Get the average voltage
      motor_data.push_back(averageCurrent()); // Get the average current
      motor_data.push_back(averageTorx()); // Get the average torque
      motor_data.push_back(averageEffic()); // Get the average efficiency
    }*/
    
    // Append data to file
    File.append_Data(Brain.timer(sec), autoNum, auton_data, get_Motor_Data());
    //motor_data.clear(); // Clear the motor data vector for next loop
    auton_data.clear(); // Clear the auton data vector for next loop
    task::sleep(20); // Cycle time
  }
  return 1;
}

bool set_up = true;

void pre_auton(void) {
  vexcodeInit();
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

  // Start the drive control task
  task pid(driveControl);
  driveTo(100, 0, 0, 10000000);
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
  //runControl = false;
  // Controller joystick variables
  short forward = Ctrl.Axis3.value(); // Gets the value of Axis 3
  short turn = Ctrl.Axis1.value(); // Gets the value of Axis 1
  
  // Motor voltage on a range of -12 to 12
  double leftVolt; // Declare left side voltage 
  double rightVolt; // Declare right side voltage

  while (1) {
    
    // Update variables
    forward = Ctrl.Axis3.value();
    turn = Ctrl.Axis1.value();

    // Calculate voltage
    leftVolt = 12 * ((forward + turn) / 100.0);
    rightVolt = 12 * ((forward - turn) / 100.0);

    // Apply leftVolt to the left motors
    leftFront.spin(fwd, leftVolt, volt);
    leftMid.spin(fwd, leftVolt, volt);
    leftBack.spin(fwd, leftVolt, volt);
    // Apply rightVolt to the right motors
    rightBack.spin(fwd, rightVolt, volt);
    rightMid.spin(fwd, rightVolt, volt);
    rightFront.spin(fwd, rightVolt, volt);

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

// Create a task for the menu handler function and set the priority low
task menuTab(MenuHandler, task::taskPrioritylow);

// Main will set up the competition functions and callbacks.
int main() {

  // Resets sensors
  calibrate_IMU();
  reset_Tracking_Wheels();

  //task odometryTask(positionTracking);

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  
  // Run the pre-autonomous function.
  pre_auton();

  // When the brain screen is released, run the fileGUIHandle function
  Brain.Screen.released(fileGUIHandle); 

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}