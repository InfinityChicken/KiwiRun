#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "main.h"

namespace kiwi {

    class Config {
        public:

            float beta;
            float zeta;

            int driving;
            int driven;

            lemlib::Chassis& chassis;
            lemlib::Drivetrain drivetrain;
            pros::MotorGroup& leftMotors;
            pros::MotorGroup& rightMotors;

            pros::Controller controller;

            std::vector<std::reference_wrapper<float>> subsysStates; //points to all subsys
            std::vector<std::string> subsysNames;
    
            Config(float beta, float zeta, int driving, int driven,
                std::vector<std::reference_wrapper<float>> subsysStates,
                std::vector<std::string> subsysNames,
                lemlib::Chassis &chassis,
                lemlib::Drivetrain drivetrain,
                pros::MotorGroup &leftMotors,
                pros::MotorGroup &rightMotors,
                pros::Controller controller);
            
            void write();
            float getWheelLinearVel(float rpm);
            float getChassisLinearVel(float leftRPM, float rightRPM);
            float getChassisAngularVel(float leftRPM, float rightRPM);
    };
    
}

#endif
