// Add definition guards
#pragma once
#ifndef PRINTING_HPP_ // If the header file has not been defined
#define PRINTING_HPP_ // Define the header file


    // Include dependencies
    #include "vex.h"
    #include "SWWight/SWWight.h"
    
    #define LCD Brain.Screen // Abbreviates Brain.Screen to just LCD
    #define BLUE "#004aad" // Abbreviates the hex code for our background color to BLUE 
    
    

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
             * @param rad   The current angle heading that Odometry function outputs (In radiants)
             * @param Left    The left rotation encoder
             * @param Middle  The Middle (perpendicular) rotation encoder 
             * @param Right   The Right rotation encoder 
            */
            void printOdom(float x, float y, float rad, rotation Left, rotation Middle, rotation Right);

            /**
             * @brief Prints out debugging information for PID
             * 
             * @param inert The inertial sensor
            */
            void printPID(inertial inert);

            /**
             * @brief Shows data analytics and file information
            */
            void printFile(data_collect &Data);

            static bool enableAuton; // A logic variable for telling when viewing the auton screen 
            static bool isDeconstructed; // When the class is destructed, the value will terminate the object's ability to print

            static motor MotorList[]; // A static motor list that contains the available motors to access

            static bool enableFile;

        protected:
            
            /**
             * @brief Prints the tabs at the top of the screen
            */
            const void makeButtons();

            /**
             * @brief Clears the screen and redraws the tabs 
            */ 
            const void screenClear();

            /**
             * @brief Prints a number (char) in a translated position
             * 
             * @param x_Max The maximum x value 
             * @param y_Max The maximum y value
             * @param autoNum The character to be printed in the position 
            */
            const void printAutonNumber(uint16_t x_Max, uint16_t y_Max, char autoNum);

            /**
             * @brief Lists a few diagnostics for the motors in the motor array provided and prints them to the screen
             * 
             * @param Motor An array for motors that will be printed to the screen
             * @param size The size of the Motor array
             *  
             * @param start_X The inital X-value to start printing from
             * @param start_Y The inital Y-value to start printing from 
            */
            const void listMotors(vex::motor Motor[], uint8_t size, uint16_t start_X, uint16_t start_Y);

    };

    /**
     * @brief Checks if a pressed x value is where the Tabs are
     * 
     * @param Menu A pointer to a LCD_Menu object 
     * 
     * @return Returns an 0
    */
    extern int checkPressedTab(LCD_Menu &Menu);

    /**
     * @brief Checks if a pressed x and y value is where the autonomous selection are
     * 
     * @param Menu A pointer to a LCD_Menu object
     * 
     * @param pressed_X The X-coordinate that the screen was pressed at 
     * @param pressed_Y The Y-coordinate that the screen was pressed at
     * 
     * @return Returns an integer based on the selected Autonomous program 
    */
    extern uint8_t checkPressedAuton(LCD_Menu &Menu, int16_t pressed_X, int16_t pressed_Y);

    extern void checkPressedFile(data_collect &Data, int16_t pressed_X, int16_t pressed_Y);


#endif // End of definition 