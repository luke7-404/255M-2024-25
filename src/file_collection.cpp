// include header
#include "file_collect.hpp"
#include "vex.h"

// Using standard libs fstream and iostream
using namespace std;


bool data_File::isNameCreated = false; // set the name as not created
bool data_File::emphasized = false; // set the name as not emphasized

// declare vars
string data_File::createdName; // the name of the created file
short data_File::currentFileNum; // the number of the current file
bool data_File::isSDInserted; // is the SD card inserted in the brain

// The constructor for the data_File class
data_File::data_File(bool SDInserted){
  this->isSDInserted = SDInserted; // set the bool
  
  // If the SDcard is inserted in the brain, a file is made
  if(SDInserted){
    cout << "File Object created" << endl; // Print that the file obj was created for debugging
    createdName = this->nameFile(); // assign the file name to the output of the nameFile function
    //cout << createdName << endl; // Print file name for debugging

  } else this->~data_File(); // If the SDcard is not in the brain then we destroy the object 
}


// Deconstructor
data_File::~data_File(void){
  // debug message: let me know if object hit error
  cout << "File object was destroyed or not created" << endl;
}

// Creates the file name
const string data_File::nameFile(void){
  
  // Creates an fstream file to receive the current file index 
  // (opens in read mode)
  fstream file("currentFile.txt", ios::in);

  // Checks if the file was opened
  if (file.is_open()){

    // Read the first line of the file
    string line; // declare line
    file >> line; // puts the contents of the file in the string
    file.close(); // Closes and saves the file
    //cout << line << endl; // Prints the line that was read

    // Converts the line read to an int
    currentFileNum = atoi(line.c_str());

    // Opens the file in write mode and truncates it (clears out the file)
    file.open("currentFile.txt", ios::out | ios::trunc);

    // Increment then write back for the next file
    file << currentFileNum+1;
    
    file.close(); // Close and save the file

    return "CSV_Data_File_" + line + ".csv"; // Return the file name string
    
  } else cout << "File did NOT open" << endl; // Output if the file did not open
  
  return "error"; // If it did not return sooner, it returns error as the string
}

// Creates a file and writes the header row of a CSV file with specific data fields.
void data_File::createFile(void){

  // make a file object that writes to the active file
  ofstream dataFile(this->createdName, ios::out);
    
  // Check if the ofstream file opened
  if(dataFile.is_open()){
    isNameCreated = true; // let the program know that the name was created
    
    // write the first row of the CSV
    dataFile << "Total Time," << "Auton Selected,"
            // PID
             << "Error," << "Prev Error," << "Integral," << "Derivative," << "Drive Power,"
             << "Turn Error," << "Prev Turn Error," << "Turn Integral," << "Turn Derivative," << "Turn Power,"
            // Odom
             << "Left TW," << "Prev Left TW," << "Delta Dist L,"
             << "Right TW," << "Prev Right TW," << "Delta Dist R,"
             << "Side TW," << "Prev Side TW," << "Delta Dist S,"
             << "Inertial Sensor," << "Prev Inertial Sensor," << "Delta Theta,"
             << "avgThetaForArc,"
             << "Delta X Local," << "Delta X Global," << "X Pos Global,"
             << "Delta Y Local," << "Delta Y Global," << "Y Pos Global,"
            // Motors
             << "AVG Temp," << "AVG Volt," << "AVG Current," << "AVG Torx," << "AVG Effic"
            << "\n";

    
    dataFile.close(); // Closes and saves the file
    //cout << "header written" << endl; // debug that data has been written
  } else this->~data_File(); // If the file does not open then we destroy the object
}

void data_File::emphasizeFile(void){
  
  // if the object has not been emphasized yet
  if (!this->emphasized){
    // add a tilde in front of the created name 
    this->createdName = this->createdName.insert(0, 1, '~');
    this->emphasized = true; // flip boolean value
    cout << "File emphasized!" << endl; // debug message
  }
}

void data_File::append_Data(short brain_Time, short autonSelection, std::vector<double> auton_data, std::vector<double> motor_data){

  // create file object in append mode
  ofstream file_to_add_to(this->createdName, ios::app);

  // check if file opened
  if(file_to_add_to.is_open()){
    // Append the time and auton selection
    file_to_add_to << Brain.timer(sec) << "," << autonSelection << ",";
    
    // Append the floored data
    for (auto &&i : auton_data) file_to_add_to << floor(i * 10000) / 10000 << ","; // auton
    for (auto &&i : motor_data) file_to_add_to << floor(i * 10000) / 10000 << ","; // motor
    
    // move to the next line when done
    file_to_add_to << "\n";

    file_to_add_to.close(); // save and close file
  }
}
