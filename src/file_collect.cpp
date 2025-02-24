// include header
#include "file_collect.hpp"

// Using standard libs fstream and iostream
using namespace std;


bool file::data_File::isNameCreated = false; // set the name as not created
bool file::data_File::emphasized = false; // set the name as not emphasized

// declare vars
string file::data_File::createdName; // the name of the created file
short file::data_File::currentFileNum; // the number of the current file
bool file::data_File::isSDInserted; // is the SD card inserted in the brain

// The constructor for the data_File class
file::data_File::data_File(lemlib::Chassis* chassisData, 
                            vector<reference_wrapper<pros::AbstractMotor>> &motors){
  this->chassisData = chassisData;
  this->motors = motors;
  this->isSDInserted = pros::usd::is_installed();
  
  // If the SDcard is inserted in the brain, a file is made
  if(this->isSDInserted){
    cout << "File Object created" << endl; // Print that the file obj was created for debugging
    this->createdName = this->nameFile(); // assign the file name to the output of the nameFile function
    cout << createdName << endl; // Print file name for debugging

  } else {
    // If the SDcard is not in the brain then we destroy the object
    cout << "File object was destroyed or not created" << endl;
  }  
}

// Creates the file name
const string file::data_File::nameFile(void){
  
  // Creates an fstream file to receive the current file index 
  // (opens in read mode)
  fstream file(FILE_INDICES_TRACKER_PATH, ios::in);

  // Checks if the file was opened
  if (file.is_open()){

    // Read the first line of the file
    string line; // declare line
    file >> line; // puts the contents of the file in the string
    file.close(); // Closes and saves the file
    //cout << line << endl; // Prints the line that was read

    // Converts the line read to an int
    this->currentFileNum = atoi(line.c_str());

    // Opens the file in write mode and truncates it (clears out the file)
    file.open(FILE_INDICES_TRACKER_PATH, ios::out | ios::trunc);

    // Increment then write back for the next file
    file << this->currentFileNum + 1;
    
    file.close(); // Close and save the file

    return this->emphasizeFile("CSV_Data_File_" + line + ".csv"); // Return the file name string
    
  } else cout << "File did NOT open" << endl; // Output if the file did not open
  
  return "error"; // If it did not return sooner, it returns error as the string
}

string file::data_File::emphasizeFile(string name){
  // Check if the field control is enabled and the file has not been emphasized
  if(pros::competition::is_field_control() && !this->emphasized){
    this->emphasized = true; // flip boolean value
    cout << "File emphasized!" << endl; // debug message
    name = name.insert(0, 1, '@'); // insert an at sign at the beginning of the name
  }

  return "/usd/" + name; // return the name and the file path destination
} 

// Creates a file and writes the header row of a CSV file with specific data fields.
void file::data_File::createFile(void){

  if(!this->isNameCreated){
    // make a file object that writes to the active file
    ofstream dataFile(this->createdName, ios::out);
       
    // Check if the ofstream file opened
    if(dataFile.is_open()){
      this->isNameCreated = true; // let the program know that the name was created
       
      // write the first row of the CSV
      dataFile << "Total Time,Auton Selected,Theta,X,Y," // General data
        /* Average of all motors */ << "AVG Temp (C),AVG Velocity,AVG Torque (Nm),AVG Voltage (V),AVG Power (W),AVG efficiency%," // Average of all motors
        /* LadyBrown */             << "LB Temp (C),LB Velocity,LB Torque (Nm),LB Voltage (V),LB Power (W),LB efficiency%," // Ladybrown
        /* Left Front Drive */      << "Left Front Temp (C),Left Front Velocity,Left Front Torque (Nm),Left Front Voltage (V),Left Front Power (W),Left Front efficiency%," 
        /* Left Mid Drive */        << "Left Mid Temp (C),Left Mid Velocity,Left Mid Torque (Nm),Left Mid Voltage (V),Left Mid Power (W),Left Mid efficiency%," 
        /* Left Back Drive */       << "Left Back Temp (C),Left Back Velocity,Left Back Torque (Nm),Left Back Voltage (V),Left Back Power (W),Left Back efficiency%," 
        /* Right Front Drive */     << "Right Front Temp (C),Right Front Velocity,Right Front Torque (Nm),Right Front Voltage (V),Right Front Power (W),Right Front efficiency%," 
        /* Right Mid Drive */       << "Right Mid Temp (C),Right Mid Velocity,Right Mid Torque (Nm),Right Mid Voltage (V),Right Mid Power (W),Right Mid efficiency%," 
        /* Right Back Drive */      << "Right Back Temp (C),Right Back Velocity,Right Back Torque (Nm),Right Back Voltage (V),Right Back Power (W),Right Back efficiency%," 
        /* First Stage Intake */    << "1st Intake Temp (C),1st Intake Velocity,1st Intake Torque (Nm),1st Intake Voltage (V),1st Intake Power (W),1st Intake efficiency%," 
        /* Second Stage Intake */   << "2nd Intake Temp (C),2nd Intake Velocity,2nd Intake Torque (Nm),2nd Intake Voltage (V),2nd Intake Power (W),2nd Intake efficiency%" 
        << "\n";        
    
      dataFile.close(); // Closes and saves the file
      cout << "header written" << endl; // debug that data has been written
    }  else {
      // If the SDcard is not in the brain then we destroy the object
      cout << "File object was destroyed or not created" << endl;
    } 
  } 
}

void file::data_File::append_Data(string autonSelection){

  // Open file in append mode
  ofstream file_to_add_to(this->createdName, ios::app);

  // check if file opened successfully
  if(file_to_add_to.is_open()){
    // Append the time and auton selection
    file_to_add_to << pros::millis()/1000 << "," << autonSelection << "," 
                   << std::setprecision(5) << chassisData->getPose().x << ","  
                   << std::setprecision(5) << chassisData->getPose().y << "," 
                   << std::setprecision(5) << chassisData->getPose().theta << ",";

    // Append the floored data
    for (auto &data : this->motorData()) file_to_add_to << std::setprecision(4) << floorf(data * 10000) / 10000 << ",";      
    
    // move to the next line when done
    file_to_add_to << "\n";

    file_to_add_to.close(); // save and close file
  }
}

vector<double> file::data_File::motorData() {
  // Create a vector to store the summed motor data, initialized with zeros
  vector<double> motorData(6, 0.0);
  // Create a vector to store the individual motor data
  vector<double> individualMotorData;

  // Iterate through the motors
  for (auto &motorRef : this->motors) {
    auto &motor = motorRef.get();
    // Iterate through the ports of the motor
    for (short i = 0; i < motor.get_port_all().size(); i++) {
      // Sum the motor data
      motorData[0] += motor.get_temperature_all()[i];
      motorData[1] += motor.get_actual_velocity_all()[i];
      motorData[2] += motor.get_torque_all()[i];
      motorData[3] += motor.get_voltage_all()[i] / 1000;
      motorData[4] += motor.get_power_all()[i];
      motorData[5] += motor.get_efficiency_all()[i];

      // Append the individual motor data
      individualMotorData.push_back(motor.get_temperature_all()[i]);
      individualMotorData.push_back(motor.get_actual_velocity_all()[i]);
      individualMotorData.push_back(motor.get_torque_all()[i]);
      individualMotorData.push_back(motor.get_voltage_all()[i]/1000);
      individualMotorData.push_back(motor.get_power_all()[i]);
      individualMotorData.push_back(motor.get_efficiency_all()[i]);
    }
  }

  // Divide the summed motor data by the number of motors to get the average
  for (int i = 0; i < motorData.size(); i++) {
    motorData[i] /= this->NUM_MOTORS;
  }

  // Append the individual motor data to the summed motor data
  motorData.insert(motorData.end(), individualMotorData.begin(), individualMotorData.end());

  return motorData; // Return the vector
}