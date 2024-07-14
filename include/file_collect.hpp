#pragma once
#ifndef FILE_COLLECT_HPP_
#define FILE_COLLECT_HPP_

// include dependencies
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <math.h>
#include <vector>

/**
 * @brief Use the data_collect class to interact with file and data collection 
 * 
 */
class data_collect {
    
    protected:
    
        static std::string createdName; // Static String that stores the generated file name
        /**
         * @brief A string type function that generates a file name when called
         * 
         * @return const string 
         */
        const std::string nameFile(void);
        
    public:

        /**
         * @brief Construct a new data collect object
         * 
         * @param SDInserted Is the SD card inserted into the brain
         */
        data_collect(bool SDInserted);

        /**
         * @brief Destroy the data collect object
         * 
         */
        ~data_collect(void);

        /**
         * @brief Create a file and prepare for file use 
         * 
         */
        void createFile(void);

        /**
         * @brief emphasize file name so it is easier to discover
         * 
         */
        void emphasizeFile(void);

        /**
         * @brief A function that adds data to the created file 
         * 
         * @param brain_Time The brain's timer value to get a base data point
         * @param pid_Data A 2D array that contains the values from the PID
         * 
         * @note for pid_Data: [0][0-4] is for lateral values
         *                     [1][0-4] is for rotational values
         */
        void add_Data(short brain_Time, std::vector<double> pid_Data);

        static bool isNameCreated; // Is the file name created
        static bool emphasized; // Is the file name emphasized
        static bool isSDInserted; // Is the SD Card inserted into the Brain
        static short currentFileNum; // The number that the files get named with
};


#endif  