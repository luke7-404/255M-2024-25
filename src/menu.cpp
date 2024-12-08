#include "menu.hpp" // Include header file

using namespace std; // Using standard for iostream

bool LCD_Menu::enableAuton = true; // Set auton Screen as default
//bool LCD_Menu::enableFile = false; // Set File Screen not as default
bool LCD_Menu::isDeconstructed = false; // Set the object as not destroyed

// Constructor
LCD_Menu::LCD_Menu(){
  cout << "LCD Menu Object was created" << endl; // Show that the object was created
  LCD::set_pen(pros::Color::white);
  LCD::set_eraser(BLUE);
}

// Deconstructor
LCD_Menu::~LCD_Menu(){
  cout << "LCD Menu Object was destroyed" << endl; // Show that the object was destroyed
  LCD::set_eraser(pros::Color::black);
  LCD::erase(); // Clear the screen
  isDeconstructed = true; // Terminate the UI button's pressing logic
}

// A function that draws the text for the Tabs at the top of the screen
void LCD_Menu::makeButtons(){

  // Prints Auton in the middle of the lines
  LCD::draw_line(0, 0, 0, 26); // Draw a divider line
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 23, 18, "Auton");
  LCD::draw_line(96, 0, 96, 26); // Draw a divider line

  // Prints System in the middle of the lines
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 115, 18, "System");
  LCD::draw_line(192, 0, 192, 26); // Draw a divider line
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 221, 18, "Odom"); // Prints Odom in the middle of the lines
  LCD::draw_line(288, 0, 288, 26); // Draw a divider line
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 322, 18, "PID"); // Prints PID in the middle of the lines
  LCD::draw_line(384, 0, 384, 26); // Draw a divider line
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 407, 18, "Files"); // Prints Files in the middle of the lines
  LCD::draw_line(479, 0, 479, 26); // Draw a divider line
}

// A function that clears the screen and redraws the Tabs and background
void LCD_Menu::screenClear(){
  LCD::erase(); // Clear the screen
  LCD::set_pen(BLUE);
  LCD::fill_rect(0, 26, 480, 214);
  //LCD::draw_rect(0, 26, 480, 214); // Draws the background
  LCD::set_pen(pros::Color::white);
  this->makeButtons(); // Draws the tabs
}

// A function that translates a character into the bottom right corner
/*void LCD_Menu::printAutonNumber(uint16_t x_Max, uint16_t y_Max, char autoNum){
  LCD.setFont(mono15); // Set font size to 15

  // print auton number in the bottom right side of the box
  LCD.printAt(x_Max-10, y_Max-5, "%c", autoNum);

  // Resets text color and font
  LCD.setPenColor(vex::white); // Sets the pen color to white
  LCD.setFont(mono20); // Sets the font size to default
}*/

// Prints out buttons for auton selection
void LCD_Menu::printAuton(){
  this->screenClear(); // Clears screen to draw shapes  

  // First row Rectangles
  LCD::draw_rect(20, 37, 105, 60); // First box
  LCD::set_pen(pros::Color::orange); // set text color orange
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 53, 71, "None"); // print none in box
  //this->printAutonNumber(125, 97, '0'); // Prints 0 in the bottom right corner of the box
  
  LCD::draw_rect(135, 37, 105, 60); // Second box
  LCD::set_pen(pros::Color::green); // set text color green
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 159, 71, "Skills"); // print skills in box
  //this->printAutonNumber(240, 97, '1'); // Prints 1 in the bottom right corner of the box

  // Second row Rectangles
  LCD::draw_rect(20, 102, 105, 60); // Third box
  LCD::set_pen(pros::Color::red); // set text color red
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 54, 137, "AWP1"); // print AWP1 in box
  //this->printAutonNumber(125, 162, '2'); // Prints 2 in the bottom right corner of the box

  LCD::draw_rect(135, 102, 105, 60); // Fourth box
  LCD::set_pen(pros::Color::blue); // set text color blue
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 169, 137, "AWP2"); // print AWP2 in box
  //this->printAutonNumber(240, 162, '3'); // Prints 3 in the bottom right corner of the box

  // Third row Rectangles
  LCD::draw_rect(20, 167, 105, 60); // Fifth box
  LCD::set_pen(pros::Color::red); // set text color red 
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 27, 201, "Goal\nRush"); // print goal rush in box
  //this->printAutonNumber(125, 227, '4'); // Prints 4 in the bottom right corner of the box
  
  LCD::draw_rect(135, 167, 105, 60); // Sixth box
  LCD::set_pen(pros::Color::blue); // set text color blue
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 144, 201, "Ring\nRush"); // print ring rush in box
  //this->printAutonNumber(240, 227, '5'); // Prints 5 in the bottom right corner of the box

  LCD::set_pen(pros::Color::white);
}

/* 
  A function that will iterate through an array of type motor and print the motors and multiple attributes of them
*/  
/*void LCD_Menu::listMotors(uint16_t start_X, uint16_t start_Y){
  LCD.drawRectangle(start_X-5, start_Y-17, 398, 162); // Draws a surrounding box for the area

  // Iterates through the Motor array
  for (short i = 0; i < 8; i++) {
    if(i == 7) break;
    // Print Statement
    /*
      The format:

      Motor(PORT#): Temp#, Voltage#, Current#, Torx#, Effic.#
    
    LCD.printAt(start_X, start_Y, false, "Motor%d: %.1f, %.2f, %.2f, %.2f, %.2f", 
    MotorList[i].index()+1, MotorList[i].temperature(celsius), MotorList[i].voltage(volt), 
    MotorList[i].current(amp), MotorList[i].torque(), MotorList[i].efficiency(pct));

    start_Y += 20; // Shifts to the next line down
  }
}*/

// Prints out system information like motor diagnostics and battery %
/*void LCD_Menu::printSystem(){
  
  /* Static Elements
  this->screenClear(); // Clears the screen
  LCD.printAt(30,60, false, "Motors: Temp, Volt, Current, Torx, Effic."); // Prints out the format for the Motor List

  /* Elements Need to Update 

  // Prints out battery information and if an SD_Card is inserted
  LCD.printAt(70, 42, false, "Battery:%d Battery_Voltage:%.3f", Brain.Battery.capacity(), Brain.Battery.voltage());
  
  // Calls the listMotors function
  this->listMotors(39, 83);
}*/
/*
// Prints out a picture of the vex field with odometry debugging
void LCD_Menu::printOdom(float x, float y, float heading){
  this->screenClear(); // Clears screen

  LCD.printAt(30, 68, false, "X(in.):%.2f", x); // Prints the X position from Odometry
  LCD.printAt(30, 88, false, "Y(in.):%.2f", y); // Prints the Y position from Odometry
  LCD.printAt(30, 107, false, "Angle(deg):%.2f", reduce_negative_180_to_180(radToDeg(heading))); // Prints the angle position from Odometry
  
  // Prints the rotation encoder position
  LCD.printAt(30, 156, false, "X-Track:%.2f", XTrack.position(rotationUnits::deg));
  LCD.printAt(30, 176, false, "Y-Track:%.2f", YTrack.position(rotationUnits::deg));
}

// construct panel/ rotational indicator
panel turnPanel(80, 41, 48, 0, 360);

// Prints out debugging information for PID
void LCD_Menu::printPID(PID_Data &drive, float heading){
  this->screenClear(); // Clears screen
  
  LCD.setPenWidth(4); // Makes the width of shapes bolder

  // Rotational circle indicator
  turnPanel.set_panel_data(heading); // add data too indicator
  turnPanel.set_data_label_enable(false); // turn off indicator text
  turnPanel.set_background_color(000,000,000); //fix GUI colors
  turnPanel.display(); // print the indicator

  // Lateral arrow indicator
  LCD::draw_line(396, 78, 430, 89); // Top line
  LCD::draw_line(280, 89, 430, 89); // Middle line
  LCD::draw_line(396, 100, 430, 89); // Bottom line

  LCD.setPenWidth(1); // Resets the pen width

  /* Elements that need to be updated 

  // Prints the Rotational information
  LCD.printAt(41, 163, false, "Heading(DEG): %.2f", heading);
  LCD.printAt(66, 182, false, "T Error: %.2f", drive.turn_Error);
  LCD.printAt(66, 201, false, "T Integ: %.2f", drive.turn_Integral);
  LCD.printAt(66, 221, false, "T Deriv: %.2f", drive.turn_Derivative);
  
  // Prints the Lateral information
  //LCD.printAt(275, 163, false, "Direction");
  LCD.printAt(295, 182, false, "Error: %.2f", drive.error);
  LCD.printAt(295, 201, false, "Integ: %.4f", drive.integral);
  LCD.printAt(295, 221, false, "Deriv: %.2f", drive.derivative);
}


// Shows file information

void LCD_Menu::printFile(data_File &Data){
  this->screenClear(); // Clears screen

  LCD.printAt(145, 50, false, "0 = false; 1 = true");
  LCD.printAt(58, 82, false, "SDCard Inserted?:%d", Brain.SDcard.isInserted());
  LCD.printAt(270, 82, false, "File Created?:%d", Data.isNameCreated);
  LCD.setFont(mono30);
  LCD.printAt(140, 120, false, "File Number:%d", Data.currentFileNum+Data.isNameCreated);
  LCD.setFont(mono20);

  // Draw emphasized box and print text 
  LCD.drawRectangle(40, 165, 397, 38);
  LCD.printAt(155, 188, false, "Mark Active File");
}

void checkPressedTab(int32_t pressed_X,LCD_Menu& Menu, PID_Data& PID, Odom_Data& Odom, data_File& File){
  // Checks if a pressed x value is where the Tabs are
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
    Menu.printOdom(Odom.xPosGlobal, Odom.yPosGlobal, Odom.currentTheta);
    Menu.enableFile = false; // Toggles file buttons functionality to off
    Menu.enableAuton = false; // Toggles auton buttons functionality to off

  } else if (pressed_X > 288 && pressed_X <= 384) {
    Menu.printPID(PID, reduce_0_to_360(Inert.rotation(rotationUnits::deg)) ); // Prints the PID information page
    Menu.enableFile = false; // Toggles file buttons functionality to off
    Menu.enableAuton = false; // Toggles auton buttons functionality to off

  } else if (pressed_X > 384) {
    Menu.printFile(File); // Prints File/ graph information page
    Menu.enableFile = true; // Toggles file buttons functionality to on
    Menu.enableAuton = false; // Toggles auton buttons functionality to off
  
  }
}


// Checks if a pressed x and y value is where the autonomous selection are
short checkPressedAuton(LCD_Menu &Menu, int16_t pressed_X, int16_t pressed_Y){
  
  // Check for selection of autonomous programs in the first row
  if (pressed_Y >= 37 && pressed_Y <= 97) {
    Menu.printAuton(); // Refresh the screen

    // Check for selection of None
    if (pressed_X >= 20 && pressed_X <= 125) {
      LCD.printAt(293, 50, false, "Selected:None");
      return 0;
    }
    // Check for selection of Skills
    else if (pressed_X >= 135 && pressed_X <= 240) {
      LCD.printAt(286, 50, false, "Selected:Skills");
      return 1;
    }
  }
  // Check for selection of autonomous programs in the second row
  else if (pressed_Y >= 102 && pressed_Y <= 162) {
    Menu.printAuton(); // Refresh the screen

    // Check for selection of AWP1
    if (pressed_X >= 20 && pressed_X <= 125) {
      LCD.printAt(294, 50, false, "Selected:AWP1");
      return 2;
    }
    // Check for selection of AWP2
    else if (pressed_X >= 135 && pressed_X <= 240) {
      LCD.printAt(294, 50, false, "Selected:AWP2");
      return 3;
    }
  }
  // Check for selection of autonomous programs in the third row
  else if (pressed_Y >= 167 && pressed_Y <= 227) {
    Menu.printAuton(); // Refresh the screen
    
    // Check for selection of Goal Rush
    if (pressed_X >= 20 && pressed_X <= 125) {
      LCD.printAt(271, 50, false, "Selected:Goal Rush");
      return 4;
    }
    // Check for selection of Ring Rush
    else if (pressed_X >= 135 && pressed_X <= 240) {
      LCD.printAt(271, 50, false, "Selected:Ring Rush");
      return 5;
    }
  }
  
  return 0; // Return 0 if no auton is selected
}

// Checks if a pressed x and y value is where the emphasized button is
void checkPressedFile(data_File &Data, int16_t pressed_X, int16_t pressed_Y){
  // check Y range
  if(pressed_Y >= 165 && pressed_Y <= 203){
    // check X range
    if(pressed_X >= 40 && pressed_X <= 397 && !Data.emphasized){
      Data.emphasizeFile(); // call the emphasized function
    }
  }
}*/
