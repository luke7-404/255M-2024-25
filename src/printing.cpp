#include "printing.hpp" // Include header file


using namespace std; // Using standard for iostream


// Add the motors to the MotorList array
motor LCD_Menu::MotorList[] = {TestMotor1, TestMotor2, TestMotor3, TestMotor4, TestMotor5, TestMotor6, TestMotor7, TestMotor8};

bool LCD_Menu::enableAuton = true; // Set auton Screen as default
bool LCD_Menu::enableFile = false; // Set File Screen not as default
bool LCD_Menu::isDeconstructed = false; // Set the object as not destroyed

// Constructor
LCD_Menu::LCD_Menu(){
  cout << "LCD Menu Object was created" << endl; // Show that the object was created
}

// Deconstructor
LCD_Menu::~LCD_Menu(){
  cout << "LCD Menu Object was destroyed" << endl; // Show that the object was destroyed
  LCD.clearScreen(); // Clear the screen
  isDeconstructed = true; // Terminate the UI button's pressing logic
}

// A constant function that draws the text for the Tabs at the top of the screen
const void LCD_Menu::makeButtons(){
  LCD.drawLine(0, 0, 0, 26); // Draw a divider line
  LCD.printAt(23, 18, "Auton"); // Prints Auton in the middle of the lines
  
  LCD.drawLine(96, 0, 96, 26); // Draw a divider line
  LCD.printAt(115, 18, "System"); // Prints System in the middle of the lines
  
  LCD.drawLine(192, 0, 192, 26); // Draw a divider line
  LCD.printAt(221, 18, "Odom"); // Prints Odom in the middle of the lines
  
  LCD.drawLine(288, 0, 288, 26); // Draw a divider line
  LCD.printAt(322, 18, "PID"); // Prints PID in the middle of the lines
  
  LCD.drawLine(384, 0, 384, 26); // Draw a divider line
  LCD.printAt(407, 18, "Graph"); // Prints Graph in the middle of the lines
  LCD.drawLine(479, 0, 479, 26); // Draw a divider line
}

// A constant function that clears the screen and redraws the Tabs and background
const void LCD_Menu::screenClear(){
  LCD.clearScreen(); // Clear the screen
  LCD.drawRectangle(0, 26, 480, 214, BLUE); // Draws the background
  this->makeButtons(); // Draws the tabs
  
}

// A constant function that translates a character into the bottom right corner
const void LCD_Menu::printAutonNumber(uint16_t x_Max, uint16_t y_Max, char autoNum){
  LCD.setFont(mono15); // Set font size to 15

  // print auton number in the bottom right side of the box
  LCD.printAt(x_Max-10, y_Max-5, "%c", autoNum);

  // Resets text color and font
  LCD.setPenColor(vex::white); // Sets the pen color to white
  LCD.setFont(mono20); // Sets the font size to default
}

// Prints out buttons for auton selection
void LCD_Menu::printAuton(){
  this->screenClear(); // Clears screen to draw shapes  

  // First row Rectangles
  LCD.drawRectangle(20, 37, 105, 60); // First box
  LCD.setPenColor(orange); // set text color orange
  LCD.printAt(53, 71, "None"); // print none in box
  this->printAutonNumber(125, 97, '0'); // Prints 0 in the bottom right corner of the box
  
  LCD.drawRectangle(135, 37, 105, 60); // Second box
  LCD.setPenColor(green); // set text color green
  LCD.printAt(159, 71, "Skills"); // print skills in box
  this->printAutonNumber(240, 97, '1'); // Prints 1 in the bottom right corner of the box

  // Second row Rectangles
  LCD.drawRectangle(20, 102, 105, 60); // Third box
  LCD.setPenColor(red); // set text color red
  LCD.printAt(54, 137, "AWP1"); // print AWP1 in box
  this->printAutonNumber(125, 162, '2'); // Prints 2 in the bottom right corner of the box

  LCD.drawRectangle(135, 102, 105, 60); // Fourth box
  LCD.setPenColor(blue); // set text color blue
  LCD.printAt(169, 137, "AWP2"); // print AWP2 in box
  this->printAutonNumber(240, 162, '3'); // Prints 3 in the bottom right corner of the box

  // Third row Rectangles
  LCD.drawRectangle(20, 167, 105, 60); // Fifth box
  LCD.setPenColor(red); // set text color red 
  LCD.printAt(27, 201, "Goal\nRush"); // print goal rush in box
  this->printAutonNumber(125, 227, '4'); // Prints 4 in the bottom right corner of the box
  
  LCD.drawRectangle(135, 167, 105, 60); // Sixth box
  LCD.setPenColor(blue); // set text color blue
  LCD.printAt(144, 201, "Ring\nRush"); // print ring rush in box
  this->printAutonNumber(240, 227, '5'); // Prints 5 in the bottom right corner of the box
}

/* 
  A function that will iterate through an 
  array of type motor and print the motors 
  and multiple attributes of them
*/  
const void LCD_Menu::listMotors(vex::motor Motor[], uint8_t size, uint16_t start_X, uint16_t start_Y){
  LCD.drawRectangle(start_X-5, start_Y-17, 398, 162); // Draws a surrounding box for the area

  // Iterates through the Motor array
  for (short i = 0; i < size; i++) {
    //cout << Motor[i].index()+1 << endl; // Outputs the motor port

    // Print Statement
    /*
      The format:

      Motor(PORT#): Temp#, Voltage#, Current#, Torx#, Effic.#
    */
    LCD.printAt(start_X, start_Y, false, "Motor%d: %.1f, %.3f, %.2f, %.2f, %.4f", 
    
    Motor[i].index()+1, Motor[i].temperature(pct), Motor[i].voltage(volt), 
    Motor[i].current(amp), Motor[i].torque(), Motor[i].efficiency(pct));

    start_Y += 20; // Shifts to the next line down
  }
  
}

// Prints out system information like motor diagnostics and battery %
void LCD_Menu::printSystem(){
  
  /* Static Elements */
  this->screenClear(); // Clears the screen
  LCD.printAt(30,60, false, "Motors: Temp, Volt, Current, Torx, Effic."); // Prints out the format for the Motor List

  /* Elements Need to Update */

  // Prints out battery information and if an SD_Card is inserted
  LCD.printAt(70, 42, false, "Battery:%d Battery_Voltage:%.3f", Brain.Battery.capacity(), Brain.Battery.voltage());
  
  // Calls the listMotors function
  this->listMotors(MotorList, sizeof(MotorList)/sizeof(MotorList[0]), 39, 83);
}

// Prints out a picture of the vex field with odometry debugging
void LCD_Menu::printOdom(float x, float y, float rad, rotation Left, rotation Middle, rotation Right){
  this->screenClear(); // Clears screen

  /* Elements that need to update */
  LCD.printAt(30, 68, false, "X(in.):%.2f", x); // Prints the X position from Odometry
  LCD.printAt(30, 88, false, "Y(in.):%.2f", y); // Prints the Y position from Odometry
  LCD.printAt(30, 107, false, "Angle(RAD):%.2f", rad); // Prints the angle position from Odometry
  
  // Prints the rotation encoder position
  LCD.printAt(30, 146, false, "Left Track:%.2f", Left.position(deg));
  LCD.printAt(30, 166, false, "Middle Track:%.2f", Middle.position(deg));
  LCD.printAt(30, 186, false, "Right Track:%.2f", Right.position(deg));

  /* Static Element */
  
  // If the SD_Card is inserted in the brain
  if(Brain.SDcard.isInserted()){
    LCD.drawImageFromFile("hs_field.png", 247, 29); // Prints the field
  }
}

// construct panel/ rotational indicator
panel turnPanel(80, 41, 48, 0, 360);

// Prints out debugging information for PID
void LCD_Menu::printPID(inertial inert = Inertial){
  this->screenClear(); // Clears screen
  
  LCD.setPenWidth(4); // Makes the width of shapes bolder

  // Rotational circle indicator
  turnPanel.set_panel_data(Control1.Axis1.value()+Control1.Axis2.value()); // add data too indicator
  turnPanel.set_data_label_enable(false); // turn off indicator text
  turnPanel.set_backgroud_color(000,000,000); //fix GUI colors
  turnPanel.display(); // print the indicator

  // Lateral arrow indicator
  LCD.drawLine(396, 78, 430, 89); // Top line
  LCD.drawLine(280, 89, 430, 89); // Middle line
  LCD.drawLine(396, 100, 430, 89); // Bottom line

  LCD.setPenWidth(1); // Resets the pen width

  /* Elements that need to be updated */

  // Prints the Rotational information
  LCD.printAt(41, 163, false, "Heading(DEG): %d", Control1.Axis1.value()+Control1.Axis2.value());
  LCD.printAt(66, 182, false, "T Error: %.2f", 90.00);
  LCD.printAt(66, 201, false, "T Integ: %.2f", 90.00);
  LCD.printAt(66, 221, false, "T Deriv: %.2f", 90.00);
  
  // Prints the Lateral information
  LCD.printAt(275, 163, false, "Direction: %s", "Forward");
  LCD.printAt(295, 182, false, "Error: %.2f", 90.00);
  LCD.printAt(295, 201, false, "Integ: %.2f", 90.00);
  LCD.printAt(295, 221, false, "Deriv: %.2f", 90.00);
}

// construct graph indicator
graph dataGraph(250, 203, 200, 170, PASS);

// Shows data analytics and file information
void LCD_Menu::printFile(data_collect &Data){
  this->screenClear(); // Clears screen

  LCD.printAt(53, 82, false, "SDCard Ins?:%d", Brain.SDcard.isInserted());
  LCD.printAt(44, 115, false, "File Created?:%d", Data.isNameCreated);
  LCD.printAt(50, 148, false, "File Number:%d", Data.currentFileNum+Data.isNameCreated);

  // Draw emphasized box and print text 
  LCD.drawRectangle(23, 165, 202, 38);
  LCD.printAt(43, 188, false, "Mark Active File");
  
  // add data for x-axis and y-axis
  dataGraph.add_data(Brain.timer(sec), abs(Control1.Axis1.value()+Control1.Axis2.value()));
  dataGraph.set_x_pass_space_per_frame(10); // x-interval spacing
  dataGraph.draw_line(); // print the graph
}


// Checks if a pressed x value is where the Tabs are
int checkPressedTab(LCD_Menu &Menu){

  // Checks if the menu is not deconstructed
  if(!Menu.isDeconstructed){
    
    // Gets the X-coordinate
    int16_t pressed_X = Brain.Screen.xPosition(); 
    
    // Checks if the pressed_X value is in a certain range
    if(pressed_X <= 96){
      Menu.printAuton(); // Prints the autonomous selection page
      Menu.enableAuton = true; // Toggles auton buttons functionality to on
      Menu.enableFile = false; // Toggles file buttons functionality to off

    } else if (pressed_X > 96 && pressed_X <= 192){  
      Menu.enableFile = false; // Toggles file buttons functionality to off
      Menu.enableAuton = false; // Toggles auton buttons functionality to off
      Menu.printSystem();// Prints the System information page

    }else if (pressed_X > 192 && pressed_X <= 288) {
      // Prints the Odometry information page
      Menu.printOdom(1, 2, 3, leftTrack, middleTrack, rightTrack);
      Menu.enableFile = false; // Toggles file buttons functionality to off
      Menu.enableAuton = false; // Toggles auton buttons functionality to off

    } else if (pressed_X > 288 && pressed_X <= 384) {
      Menu.printPID(); // Prints the PID information page
      Menu.enableFile = false; // Toggles file buttons functionality to off
      Menu.enableAuton = false; // Toggles auton buttons functionality to off
  
    }
  }
  return 0;
}

// Checks if a pressed x and y value is where the autonomous selection are
uint8_t checkPressedAuton(LCD_Menu &Menu, int16_t pressed_X, int16_t pressed_Y){
  // If the Y-value is in a certain range, then check if the X-Value is in a certain range

    if (pressed_Y >= 37 && pressed_Y <= 97) {

      if (pressed_X >= 20 && pressed_X <= 125) {
        Menu.printAuton(); // Refresh the screen
        LCD.printAt(293, 50, false, "Selected:None"); // Print the selected Auton
        return 0; // Return the numerical value of the auton
      }
      else if (pressed_X >= 135 && pressed_X <= 240) {
        Menu.printAuton();
        LCD.printAt(286, 50, false, "Selected:Skills"); // Print the selected Auton
        return 1; // Return the numerical value of the auton
      }
        
    } else if (pressed_Y >= 102 && pressed_Y <= 162) {
      if (pressed_X >= 20 && pressed_X <= 125) {
        Menu.printAuton();
        LCD.printAt(294, 50, false, "Selected:AWP1"); // Print the selected Auton
        return 2; // Return the numerical value of the auton
      }
      else if (pressed_X >= 135 && pressed_X <= 240) {
        Menu.printAuton();
        LCD.printAt(294, 50, false, "Selected:AWP2"); // Print the selected Auton
        return 3; // Return the numerical value of the auton
      }
        
    } else if (pressed_Y >= 167 && pressed_Y <= 227) {
      if (pressed_X >= 20 && pressed_X <= 125) {
        Menu.printAuton();
        LCD.printAt(271, 50, false, "Selected:Goal Rush"); // Print the selected Auton
        return 4; // Return the numerical value of the auton
      }
      else if (pressed_X >= 135 && pressed_X <= 240) {
        Menu.printAuton();
        LCD.printAt(271, 50, false, "Selected:Ring Rush"); // Print the selected Auton
        return 5; // Return the numerical value of the auton
      } 
    }

  // If the function reaches here we return a 6 to symbolize that no button has been pressed
  return 6;
}

// Checks if a pressed x and y value is where the emphasized button is
void checkPressedFile(data_collect &Data, int16_t pressed_X, int16_t pressed_Y){
  
  // check Y range
  if(pressed_Y >= 165 && pressed_Y <= 203){

    // check X range
    if(pressed_X >= 23 && pressed_X <= 225 && !Data.emphasized){
      Data.emphasizeFile(); // call the emphasized function
    }

  }
}
