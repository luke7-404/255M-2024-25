// include header
#include "file_collect.hpp"

// Using standard libs fstream and iostream
using namespace std;

// set defaults
bool data_collect::isNameCreated = false;
bool data_collect::emphasized = false;

// declare vars
string data_collect::createdName; 
short data_collect::currentFileNum;
bool data_collect::isSDInserted;


// Constructor
data_collect::data_collect(bool SDInserted){
  this->isSDInserted = SDInserted;
  
  // If the SDcard is inserted in the brain, a file is made
  if(SDInserted){
    cout << "File Object created" << endl; // Print that the file obj was created for debugging
    createdName = this->nameFile(); // assign the file name to the output of the nameFile function
    //cout << createdName << endl; // Print file name for debugging

  } else this->~data_collect(); // If the SDcard is not in the brain then we destroy the object 
}


// Deconstructor
data_collect::~data_collect(void){
  // debug message: let me know if object hit error
  cout << "File object was destroyed or not created" << endl;
}

// Creates the file name
const string data_collect::nameFile(void){
  
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

    // Prints the string that was converted and the string that was written
    //cout << currentFileNum << " | " << currentFileNum+1 << endl; 
    
    file.close(); // Close and save the file

    return "CSV_Data_File_" + line + ".csv"; // Return the file name string
    
  } else cout << "File did NOT open" << endl; // Output if the file did not open
  
  return "error"; // If it did not return sooner, it returns error as the string
}

void data_collect::createFile(void){

  // make a file object that writes to the active file
  ofstream dataFile(this->createdName, ios::out);
    
  // Check if the ofstream file opened
  if(dataFile.is_open()){
    isNameCreated = true; // let the program know that the name was created
    
    // write the first row of the CSV
    dataFile << "Total Time," 
            << "Drivetrain Avg," << "Lat Error," << "Lat Deriv," << "Prev Error,"  << "Lat Mtr Pwr," 
            << "Inertial Position," << "Rot Error,"  << "Rot Deriv," << "Turn Prev Error," << "Rot Mtr Pwr," << "\n";

    
    dataFile.close(); // Closes and saves the file
    cout << "header written" << endl; // debug that data has been written
  } else this->~data_collect(); // If the file does not open then we destroy the object
}

void data_collect::emphasizeFile(void){
  
  // if the object has not been emphasized yet
  if (!this->emphasized){
    // add a tilde in front of the created name 
    this->createdName = this->createdName.insert(0, 1, '~');
    this->emphasized = true; // flip boolean value
    cout << "File emphasized!" << endl; // debug message
  }
}

void data_collect::add_Data(short brain_Time, double pid_Data[2][5]){

  // create file object in append mode
  ofstream file_to_add_to(this->createdName, ios::app);

  // check if file opened
  if(file_to_add_to.is_open()){
    
    // Append the time
    file_to_add_to << brain_Time << ",";

    /* Floor the double and float values to the 5th decimal place */

    /* iterate through PID data */
    // row for loop
    for (uint8_t i = 0; i < 2; i++){
      // column for loop
      for (uint8_t j = 0; j < 5; j++){
        
        // floor array elements and write to file 
        file_to_add_to << floorf(pid_Data[i][j] * 100000) / 100000 << ",";
      }   
    }

    // move to the next line when done
    file_to_add_to << "\n";

    // save and close file
    file_to_add_to.close();
  }
  cout << "data written" << endl; // debug message
}




// LAST YEAR'S CODE (eww)
/*


// And writes the data to the SD card
void addData(short driverTime, int avg, double InertPos, int latErr, double rotErr, int latDeriv,  
              double rotDeriv, int latPrevErr, double rotPrevErr, double RotMtrPwr, double LatMtrPwr){

  totalElapsedTime = Brain.Timer.value();
 
  // Calculate the average temperature of all 8 motors in Fahrenheit

  double* tempPtr = new double; // Create a temporary pointer that points to the average of all the motors
  
  *tempPtr = (leftFront.temperature(fahrenheit) + 
              leftMid.temperature(fahrenheit) + 
              leftBack.temperature(fahrenheit) + 
              rightBack.temperature(fahrenheit) + 
              rightMid.temperature(fahrenheit) + 
              rightFront.temperature(fahrenheit) + 
              Cata.temperature(fahrenheit)+
              Intake.temperature(fahrenheit))/8;

  
  float motorTempAvg = floorf(*tempPtr * 10000) / 10000; 
  delete tempPtr; // delete the pointer so that there is no memory leak
  // more flooring
  float flooredInertPos = floorf(InertPos * 10000) / 10000;
  float flooredRotErr = floorf(rotErr * 10000) / 10000;
  float flooredRotDeriv = floorf(rotDeriv * 10000) / 10000;
  float flooredRotPrevErr = floorf(rotPrevErr * 10000) / 10000;
  float flooredLMP = floorf(LatMtrPwr * 10000) / 10000;
  float flooredRMP = floorf(RotMtrPwr * 10000) / 10000;

  // Short hand variable names to compact the code
  
    l = left
    r = right
    m = mid
    b = back
    f = front
    e or Eff = efficiency
  
  float lfe, rfe, lme, rme, lbe, rbe, cataEff, intakeEff;
  
  // Updating the variables that hold the motor efficiencies
  lfe = leftFront.efficiency(pct);
  rfe = rightFront.efficiency(pct);
  lme = leftMid.efficiency(pct);
  rme = rightMid.efficiency(pct);
  lbe = leftBack.efficiency(pct);
  rbe = rightBack.efficiency(pct);
  cataEff = Cata.efficiency(pct);
  intakeEff = Intake.efficiency(pct);     

  // Use the sprintf function to append all the data to the buffer character array (string)
  sprintf(buffer,"%d, %d , %d , %.4f , %d , %d , %.4f , %d , %.4f , %d , %.4f , %d , %.4f , %.4f , %.4f, %.4f , %.4f , %.4f , %.4f , %.4f , %.4f , %.4f , %.4f\n",
   totalElapsedTime, driverTime, timesLaunched, motorTempAvg, runAuton, avg, flooredInertPos, latErr, flooredRotErr, latDeriv, 
   flooredRotDeriv, latPrevErr, flooredRotPrevErr, flooredLMP, flooredRMP, lfe, lme, lbe, rfe, rme, rbe, cataEff, intakeEff);
  
  // append the buffer to the data file
  Brain.SDcard.appendfile(fileName.c_str(), (uint8_t *)buffer, strlen(buffer));
  
}
















*/