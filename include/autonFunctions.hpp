#pragma once
#ifndef AUTONFUNCTIONS_HPP
#define AUTONFUNCTIONS_HPP

#include "main.h"
#include "lemlib/api.hpp"

using namespace pros;

// The distance sensor that detects the claw
Distance clawDistance(18);

class Auton_Functions{

    public:
        Auton_Functions(lemlib::Chassis& drive, ADIPneumatics& intakeRaiser, ADIPneumatics& mogoClaw,
                        ADIPneumatics& doinker, MotorGroup& intake);

        void autoClampClaw();
        static std::int8_t release;

        struct RED_Auton {
                RED_Auton(Auton_Functions& parent) : parent(parent){}
                void AWP1();
                void AWP2();
                void Skills();
                void goalRush();
                void ringRush();
            private:
                Auton_Functions& parent;
        };

        struct BLUE_Auton {
                BLUE_Auton(Auton_Functions& parent) : parent(parent){}
                void AWP1();
                void AWP2();
                void Skills();
                void goalRush();
                void ringRush();
            private:
                Auton_Functions& parent;
        };
        
        
    private:
        lemlib::Chassis& drive;
        ADIPneumatics& intakeRaiser; 
        ADIPneumatics& mogoClaw;
        ADIPneumatics& doinker; 
        MotorGroup& intake;
};


#endif