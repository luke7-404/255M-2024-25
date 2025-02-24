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
namespace file{
    class data_File {
        
        private:

            const std::string FILE_INDICES_TRACKER_PATH = "/usd/currentFile.txt";

            const uint8_t NUM_MOTORS = 9; // The number of motors in the system

            static std::string createdName; // Static String that stores the generated file name
            /**
             * @brief A string type function that generates a file name when called
             * 
             * @return const string 
             */
            const std::string nameFile();

            
            static bool emphasized; // Is the file name emphasized
            /**
             * @brief A function that emphasizes the file name so it is easier to discover
             * 
             * @param name The name of the file
             * @return std::string 
             */
            std::string emphasizeFile(std::string name);

            /**
             * @brief A function that gets and handles the data from the motors then returns it
             * 
             * @return std::vector<double> 
             */
            std::vector<double> motorData();
            
        public:

            /**
             * @brief Construct a new data collect object
             * 
             * @param chassisData The chassis object that contains the odometry data
             * @param motors The array that contains references to all of the motors
             */
            data_File(lemlib::Chassis* chassisData, 
                        std::vector<std::reference_wrapper<pros::AbstractMotor>> &motors);

            /**
             * @brief Create a file and prepare for file use
             */
            void createFile();


            /**
             * @brief A function that adds data to the created file 
             * 
             * @param autonSelection The autonomous selection that was made
             */
            void append_Data(std::string autonSelection);

            static bool isNameCreated; // Is the file name created
            static bool isSDInserted; // Is the SD Card inserted into the Brain
            static short currentFileNum; // The number that the files get named with
            lemlib::Chassis* chassisData; // The chassis object that contains the odometry data
            std::vector<std::reference_wrapper<pros::AbstractMotor>> motors; // The array that contains references to all of the motors
    };
};

#endif  