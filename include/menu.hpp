
// Add definition guards
#pragma once
#ifndef PRINTING_HPP_ // If the header file has not been defined
#define PRINTING_HPP_ // Define the header file

    // Include dependencies
    #include "main.h"
    
    
    // Define commonalities
    #define LCD pros::screen // Abbreviates Brain.Screen to just LCD
    #define BLUE 0x004aad // Abbreviates the hex code for our background color to BLUE 
    
    /**
     * @brief Use the LCD_Menu class to create a GUI on the VEX V5 Brain
    */
    class LCD_Menu{
        public:
            /**
             * @brief Creates a new LCD_Menu object
            */
            LCD_Menu();
            /**
             * @brief Destroys a LCD_Menu object
            */
            ~LCD_Menu();
            /**
             * @brief Prints out buttons for auton selection
            */
            void printAuton();
            /**
             * @brief Prints out system information like motor diagnostics and battery %
            */
            void printSystem();
            
            /**
             * @brief Prints out a picture of the vex field with odometry debugging
             * 
             * @param x     The current X-coordinate that Odometry function outputs (In inches)  
             * @param y     The current Y-coordinate that Odometry function outputs (In inches)
             * @param heading   The current angle heading that Odometry function outputs (In degrees)
            
            void printOdom(float x, float y, float heading);
            /**
             * @brief Prints out debugging information for PID
             * @param drive The PID_Data object
             * @param heading The current angle heading (In degrees)
            
            void printPID(PID_Data &drive, float heading);
            /**
             * @brief Shows data analytics and file information
             * @param Data The data_File object
            
            void printFile(data_File &Data);
            */
            static bool enableAuton; // A logic variable for when viewing the auton screen 
            //static bool enableFile; // A logic variable for when viewing the file screen
            static bool isDeconstructed; // When the class is destructed, the value will terminate the object's ability to print

        private:
            /**
             * @brief Prints the tabs at the top of the screen
            */
            void makeButtons();
            /**
             * @brief Clears the screen and redraws the tabs 
            */ 
            void screenClear();
            /**
             * @brief Prints a number (char) in a translated position
             * 
             * @param x_Max The maximum x value 
             * @param y_Max The maximum y value
             * @param autoNum The character to be printed in the position 
            */
            void printAutonNumber(uint16_t x_Max, uint16_t y_Max, char autoNum);
            /**
             * @brief Lists a few diagnostics for the motors in the motor array provided and prints them to the screen
             * 
             * @param start_X The inital X-value to start printing from
             * @param start_Y The inital Y-value to start printing from 
            */
            void listMotors(uint16_t start_X, uint16_t start_Y);

    };

    /**
     * @brief Checks if the pressed X value is where the Menu tabs are
     * 
     * @param pressed_X The x-coordinate that was pressed
     * @param Menu pointer to the Menu object
     * @param PID pointer to the PID object
     * @param Odom pointer to the Odom object
     * @param File pointer to the File object
    */
    extern void checkPressedTab(int32_t pressed_X,LCD_Menu& Menu/*, PID_Data& PID, Odom_Data& Odom, data_File& File*/);

    /**
     * @brief Checks if the pressed x and y value is where the auton buttons are
     * 
     * @param Menu A pointer to a LCD_Menu object
     * @param pressed_X The X-coordinate that was pressed 
     * @param pressed_Y The Y-coordinate that was pressed
     * 
     * @return Returns an integer based on the selected Autonomous program 
    */
    extern short checkPressedAuton(LCD_Menu &Menu, int16_t pressed_X, int16_t pressed_Y);
    
    /**
     * @brief Checks if the pressed x and y value is where the active file button is
     * 
     * @param Data pointer to the File object
     * @param pressed_X The X-coordinate that was pressed 
     * @param pressed_Y The Y-coordinate that was pressed
     */
    //extern void checkPressedFile(data_File &Data, int16_t pressed_X, int16_t pressed_Y);
#endif // End of header 