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
            void AWP1();
            void AWP2();
            void Skills();
            void goalRush();
            void ringRush();
        };

        struct BLUE_Auton {
            void AWP1();
            void AWP2();
            void Skills();
            void goalRush();
            void ringRush();
        };
        
        
    private:
        lemlib::Chassis& drive;
        ADIPneumatics& intakeRaiser; 
        ADIPneumatics& mogoClaw;
        ADIPneumatics& doinker; 
        MotorGroup& intake;
};


#endif