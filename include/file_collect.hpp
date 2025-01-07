#pragma once
#ifndef FILE_COLLECT_HPP_
#define FILE_COLLECT_HPP_

// include dependencies
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include "lemlib/api.hpp" // IWYU pragma: keep 
#include "main.h"

/**
 * @brief Use the data_File class to interact with file and data collection 
 * 
 */
class data_File {
    
    private:
    
        static std::string createdName; // Static String that stores the generated file name
        /**
         * @brief A string type function that generates a file name when called
         * 
         * @return const string 
         */
        const std::string nameFile();
        
    public:

        /**
         * @brief Construct a new data collect object
         * 
         * @param SDInserted Is the SD card inserted into the brain
         */
        data_File(bool SDInserted, /*lemlib::Chassis &drive,*/ 
                    std::vector<std::reference_wrapper<pros::AbstractMotor>> motors);

        /**
         * @brief Destroy the data collect object
         */
        ~data_File();

        /**
         * @brief Create a file and prepare for file use
         */
        void createFile(void);

        /**
         * @brief emphasize file name so it is easier to discover
         */
        void emphasizeFile(void);

        /**
         * @brief A function that adds data to the created file 
         * 
         * @param brain_Time The brain's timer value to get a base data point
         * @param autonSelection The number that was selected for autonomous
         * @param auton_Data An array that contains the values from PID and Odom
         * @param motor_data An array that contains averaged values from all motors 
         */
        void append_Data(short brain_Time, short autonSelection);

        static bool isNameCreated; // Is the file name created
        static bool emphasized; // Is the file name emphasized
        static bool isSDInserted; // Is the SD Card inserted into the Brain
        static short currentFileNum; // The number that the files get named with
        lemlib::Chassis &odomData;
        std::vector<std::reference_wrapper<pros::AbstractMotor>> motorData;
};


#endif  