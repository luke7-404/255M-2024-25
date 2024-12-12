#include "autonFunctions.hpp"


std::int8_t Auton_Functions::release = 1;
void Auton_Functions::autoClampClaw(){
    while (competition::is_autonomous()) {
        
             if(release == 1) { mogoClaw.retract(); }
        else if(release == -1) { mogoClaw.extend(); }
        else if(clawDistance.get_distance() > 140 && clawDistance.get_distance() < 170){
            mogoClaw.extend();
        } else mogoClaw.retract();

        delay(50);
    }
}

Auton_Functions::Auton_Functions(lemlib::Chassis& drive, ADIPneumatics& intakeRaiser, 
                                 ADIPneumatics& mogoClaw, ADIPneumatics& doinker, MotorGroup& intake)
        : drive(drive), intakeRaiser(intakeRaiser), mogoClaw(mogoClaw), doinker(doinker), intake(intake)
{
    std::cout << "Auton Func obj created and chassis obj passed" << std::endl;
}

void Auton_Functions::RED_Auton::AWP1(){
  
}

void Auton_Functions::RED_Auton::AWP2(){

}

void Auton_Functions::BLUE_Auton::AWP1(){
  
}

void Auton_Functions::BLUE_Auton::AWP2(){
    
}
