#include "menu.hpp" // Include header file

using namespace std; // Using standard for iostream

bool LCD_Menu::enableAuton = true; // Set auton Screen as default
bool LCD_Menu::isBlue = false; // Set auton Screen as default
//bool LCD_Menu::enableFile = false; // Set File Screen not as default
bool LCD_Menu::isDeconstructed = false; // Set the object as not destroyed

// Constructor
LCD_Menu::LCD_Menu(std::vector<std::reference_wrapper<pros::AbstractMotor>> &motorArray) {
  cout << "LCD Menu Object was created" << endl; // Show that the object was created
  this->motors = motorArray;
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
  LCD::set_eraser(pros::Color::black);
  // Prints Auton in the middle of the lines
  LCD::draw_line(0, 0, 0, 25.5); // Draw a divider line
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 15.5, 7.5, "Auton R");
  LCD::draw_line(96, 0, 96, 25.5); // Draw a divider line

  // Prints System in the middle of the lines
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 111, 7.5, "Auton B");
  LCD::draw_line(192, 0, 192, 25.5); // Draw a divider line
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 211.25, 7.5, "System"); // Prints Odom in the middle of the lines
  LCD::draw_line(288, 0, 288, 25.5); // Draw a divider line
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 301, 7.5, "Position"); // Prints PID in the middle of the lines
  LCD::draw_line(384, 0, 384, 25.5); // Draw a divider line
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 411.5, 7.5, "Data"); // Prints Files in the middle of the lines
  LCD::draw_line(479, 0, 479, 25.5); // Draw a divider line
}

// A function that clears the screen and redraws the Tabs and background
void LCD_Menu::screenClear(){
  LCD::erase(); // Clear the screen
  LCD::set_pen(BLUE); // set the color of the cursor
  LCD::fill_rect(0, 26, 480, 272); // Draws the background
  LCD::set_pen(pros::Color::white); // Reset the color
  this->makeButtons(); // Draws the tabs
}

// Draws the autonomous selection boxes on the screen 
void LCD_Menu::makeAutonButtons(uint16_t start_X1, uint16_t start_Y1, uint16_t start_X2, uint16_t start_Y2){
  
  // for the number of boxes (6)
  for(uint8_t i = 1; i <= 6; i++){
    LCD::set_pen(pros::Color::black); // set the fill color of the box
    
    // When i is even translate x so it is the second box in the row
    if (i % 2 == 0) {
      start_X1 += 121;
      start_X2 += 121;
    } 
    // When i is odd and is not 1, shift x to the first box of the row and then shift down
    else if ((i % 2 > 0) && i != 1){
      start_X1 -= 121;
      start_X2 -= 121;
      start_Y1 += 69;
      start_Y2 += 69;
    }
    
    LCD::fill_rect(start_X1, start_Y1, start_X2, start_Y2); // Print box
    LCD::set_pen(pros::Color::white); // set cursor color to white
    LCD::draw_rect(start_X1, start_Y1, start_X2, start_Y2); // add outline to box
  }

}

// A function that translates a character into the bottom right corner
void LCD_Menu::printAutonNumber(uint16_t x_Max, uint16_t y_Max, char autoNum){
  
  // Set font size to small
  // Translate the coordinate point to the bottom corner
  // Print the number
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, x_Max-12, y_Max-18, "%c", autoNum); 

  LCD::set_pen(pros::Color::white); // Reset the pen color to white
}

// Prints out buttons for auton selection
void LCD_Menu::printAuton(pros::Color color, std::vector<char> autoNum){
  this->screenClear(); // Clears screen to draw shapes  

  // Draw boxes
  this->makeAutonButtons(16, 32, 130, 93.75);

  // First row buttons
  LCD::set_pen(pros::Color::dark_orange); // set text color orange
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 55, 53, "None"); // print none in box
  this->printAutonNumber(130, 93.75, '0'); // Prints 0 in the bottom right corner of the box
  
  LCD::set_pen(pros::Color::green); // set text color green
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 166, 53, "Skills"); // print skills in box
  this->printAutonNumber(251, 93.75, autoNum[0]); // Prints the first char

  // Second row buttons
  LCD::set_pen(color);
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 54, 126, "AWP1"); // print AWP1 in box
  this->printAutonNumber(130, 162.75, autoNum[1]); // Prints the second char

  LCD::set_pen(color);
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 175, 126, "AWP2"); // print AWP2 in box
  this->printAutonNumber(251, 162.75, autoNum[2]); // Prints the third char

  // Third row buttons
  LCD::set_pen(color);
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 28, 194.5, "Goal Rush"); // print goal rush in box
  this->printAutonNumber(130, 231.75, autoNum[3]); // Prints the fourth char
  
  LCD::set_pen(color);
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 149.25, 194.5, "Ring Rush"); // print ring rush in box
  this->printAutonNumber(251, 231.75, autoNum[4]); // Prints the fifth char
  
  LCD::set_pen(pros::Color::white);
}

/* 
  A function that will iterate through an array of type motor and print the motors and multiple attributes of them
*/  
void LCD_Menu::listMotors(uint16_t start_X, uint16_t start_Y){
  LCD::set_pen(pros::Color::black);
  LCD::draw_rect(start_X-5, start_Y-17, 398, 162); // Draws a surrounding box for the area
  LCD::set_pen(pros::Color::white);

  for (auto &motorRef : this->motors) { // Iterates through all of the motor devices in the array
    auto &motor = motorRef.get();
    // Per device in each motor device, access data
    for (short i = 0; i < motor.get_port_all().size(); i++)
    {
      // Print Statement
      /*
        The format:

        Motor(PORT#): Temp#, Voltage#, Torx#, Effic.#
      */
      LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, start_X, start_Y, // Size and position of text
                  "Motor%d: %.1f, %.2f, %.2f, %.2f", // Text to print
                    motor.get_port_all()[i], motor.get_temperature_all()[i], // port value and motor temperature
                    motor.get_voltage_all()[i] * 1000.0, // motor voltage (converted from millivolts to volts)
                    motor.get_torque_all()[i], // motor voltage
                    motor.get_efficiency_all()[i] // motor efficiency
      );

      start_Y += 20; // Shifts to the next line down
    }
    
  }

}

// Prints out system information like motor diagnostics and battery %
void LCD_Menu::printSystem(){
  
  //Static Elements
  this->screenClear(); // Clears the screen

  // Prints out the format for the Motor List
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 30, 60, "Motors: Temp, Volt, Current, Torx, Effic."); 

  /* Elements Need to Update */

  // Prints out battery information
  LCD::print(pros::text_format_e_t::E_TEXT_MEDIUM, 70, 42, "Battery:%.1f Battery_Voltage:%.1f", 
                                                            pros::battery::get_capacity(), pros::battery::get_voltage());
  
  // Calls the listMotors function
  this->listMotors(39, 83);
}

// Prints out a picture of the vex field with odometry debugging
void LCD_Menu::printPose(float x, float y, float heading){
  this->screenClear(); // Clears screen
  LCD::set_eraser(BLUE);
  LCD::print(pros::E_TEXT_LARGE, 30, 68, "X(in.):%.2f", x); // Prints the X position from Odometry
  LCD::print(pros::E_TEXT_LARGE, 30, 98, "Y(in.):%.2f", y); // Prints the Y position from Odometry
  LCD::print(pros::E_TEXT_LARGE, 30, 128, "Angle(deg):%.2f", heading); // Prints the angle position from Odometry
}


// Shows file information

/*void LCD_Menu::printFile(data_File &Data){
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

*/

char checkout; // Helper variable that holds values for the return statement

void checkPressedTab(int32_t pressed_X, LCD_Menu& Menu, lemlib::Chassis& chassis){
  Menu.enableAuton = false; // Toggles auton buttons functionality to on
  Menu.isBlue = false;
  //Menu.enableFile = false; // Toggles file buttons functionality to off
  
  // Checks if a pressed x value is where the Tabs are
  if(pressed_X <= 96){
    Menu.enableAuton = true; // Toggles auton buttons functionality to on
    checkout = 0;
    Menu.printAuton(pros::Color::red); // Prints the autonomous selection page

  } else if (pressed_X > 96 && pressed_X <= 192){  
    Menu.enableAuton = true; // Toggles auton buttons functionality to off
    Menu.isBlue = true;// Prints the System information page
    checkout = 0;
    Menu.printAuton(pros::Color::blue, {'A', 'B', 'C', 'D', 'E'});

  } else if (pressed_X > 192 && pressed_X <= 288) {
    // Prints the system information page
    Menu.printSystem();
    //Menu.enableFile = false; // Toggles file buttons functionality to off
    Menu.enableAuton = false; // Toggles auton buttons functionality to off

  } else if (pressed_X > 288 && pressed_X <= 384) {
    Menu.printPose(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    //Menu.enableFile = false; // Toggles file buttons functionality to off
    Menu.enableAuton = false; // Toggles auton buttons functionality to off

  } else if (pressed_X > 384) {
    //Menu.printFile(File); // Prints File/ graph information page
    //Menu.enableFile = true; // Toggles file buttons functionality to on
    Menu.enableAuton = false; // Toggles auton buttons functionality to off
  
  }
}

// Checks if a pressed x and y value is where the autonomous selection are
char checkPressedAuton(LCD_Menu &Menu, int16_t pressed_X, int16_t pressed_Y){
    
  // Refresh the screen and print the tab's autons
  if(Menu.isBlue) Menu.printAuton(pros::Color::blue, {'A', 'B', 'C', 'D', 'E'});
   else Menu.printAuton(pros::Color::red); 

  LCD::set_eraser(BLUE); // Set the text background color to the default

  // Check for selection of autonomous programs in the first column
  if (pressed_X >= 15.99 && pressed_X <= 130.01) {
    // Check for selection of None
    if (pressed_Y >= 31.99 && pressed_Y <= 93.76) {
      LCD::print(pros::E_TEXT_MEDIUM, 286, 50, "Selected:None");
      return 0;
    }
    // Check for selection of AWP1
    else if (pressed_Y >= 100.99 && pressed_Y <= 162.76) {
      LCD::print(pros::E_TEXT_MEDIUM, 286, 50, "Selected:AWP1");
      checkout = 2;
    }
    // Check for selection of Goal Rush
    else if(pressed_Y >= 169.99 && pressed_Y <= 231.76){
      LCD::print(pros::E_TEXT_MEDIUM, 273, 50, "Selected:Goal Rush");
      checkout = 4;
    }
  }

  // Check for selection of autonomous programs in the second column
  else if (pressed_X >= 136.99 && pressed_X <= 251.01) {
    // Check for selection of Skills
    if (pressed_Y >= 31.99 && pressed_Y <= 93.76) {
      LCD::print(pros::E_TEXT_MEDIUM, 286, 50, "Selected:Skills");
      checkout = 1;
    }
    // Check for selection of AWP2
    else if (pressed_Y >= 100.99 && pressed_Y <= 162.76) {
      LCD::print(pros::E_TEXT_MEDIUM, 286, 50, "Selected:AWP2");
      checkout = 3;
    }
    // Check for selection of Ring Rush
    else if(pressed_Y >= 169.99 && pressed_Y <= 231.76){
      LCD::print(pros::E_TEXT_MEDIUM, 273, 50, "Selected:Ring Rush");
      checkout = 5;
    }
  } else return 0;
  
  // Return the ASCII representation of the char
  return Menu.isBlue ? (checkout+64) : (checkout+48); 
}

// Checks if a pressed x and y value is where the emphasized button is
/*void checkPressedFile(data_File &Data, int16_t pressed_X, int16_t pressed_Y){
  // check Y range
  if(pressed_Y >= 165 && pressed_Y <= 203){
    // check X range
    if(pressed_X >= 40 && pressed_X <= 397 && !Data.emphasized){
      Data.emphasizeFile(); // call the emphasized function
    }
  }
}*/
