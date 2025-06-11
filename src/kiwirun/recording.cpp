#include "kiwirun/includes.hpp"
#include "globals.hpp"
#include <fstream>
#include <stdexcept>
#include <deque> 
#include "pros/misc.hpp"

namespace kiwi {

    void Config::write() {

        pros::Task writeTask([&]() {
            nlohmann::json master = nlohmann::json::array();  // holds all points in the path

            int index = 0;
            int count = 1;

            std::deque<float> linearVelocityHistory;
            std::deque<float> angularVelocityHistory;

            const int historySize = 5; 

            // helper function to average velocity over last 5 values
            auto getAverage = [&](std::deque<float>& history, float newValue) {
                history.push_back(newValue); // add newest value

                if (history.size() > historySize) {
                    history.pop_front(); // remove oldest if too many
                }

                float total = 0;
                for (float value : history) {
                    total += value;
                }

                return total / history.size(); // return average
            };

            // wait until robot starts moving
            while(leftMotors.get_actual_velocity() < 10 && rightMotors.get_actual_velocity() < 10) {
                pros::delay(10);
            }

            // main loop to record points until controller button is pressed
            while (true) {
                nlohmann::json pointData;

                lemlib::Pose pose = this->chassis.getPose(true);
                float leftRPM = leftMotors.get_actual_velocity(0); // get left motor velocity
                float rightRPM = rightMotors.get_actual_velocity(0); // get right motor velocity

                // round pose values to 3 decimal places
                float xPos = std::round(pose.x * 1000) / 1000;
                float yPos = std::round(pose.y * 1000) / 1000;
                float thetaPos = std::round(pose.theta * 1000) / 1000;

                pointData["index"] = index++;
                pointData["x pos"] = xPos;
                pointData["y pos"] = yPos;
                pointData["theta"] = thetaPos;

                float rawLinearVel = getChassisLinearVel(leftRPM, rightRPM); // compute linear velocity
                float rawAngularVel = getChassisAngularVel(leftRPM, rightRPM); // compute angular velocity

                pointData["linear vel"] = rawLinearVel;
                pointData["angular vel"] = rawAngularVel;

                // record all subsystem state values
                for (size_t i = 0; i < subsysStates.size(); i++) {
                    pointData[subsysNames[i]] = subsysStates[i].get();
                }

                master.push_back(pointData);  // add this point to the path

                // stop recording when controller A button is pressed
                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
                    break;
                }

                pros::delay(10); // wait between samples
            }

            pros::delay(500);

            std::ofstream file("/usd/path.json", std::ios::out | std::ios::trunc); // open path file on SD

            pros::delay(500);

            if(!pros::usd::is_installed()) {
                throw std::runtime_error("No SD card present.\n"); // make sure SD card is there
            }
            
            if(!file.is_open()) {
                throw std::runtime_error("Failed to open file."); // make sure file opened
            }

            file << master.dump(3);  // write the JSON to the file
            file.close();
        });
    }

    // converts wheel rpm to linear velocity in inches/sec
    float Config::getWheelLinearVel(float rpm) {
        return rpm / 60 * M_PI * this->drivetrain.wheelDiameter * this->driving / this->driven; 
    }

    // computes average linear velocity of the chassis
    float Config::getChassisLinearVel(float leftRPM, float rightRPM) {
        float left = getWheelLinearVel(leftRPM);    
        float right = getWheelLinearVel(rightRPM);
        return (left + right) / 2;
    }

    // computes angular velocity based on left and right sides
    float Config::getChassisAngularVel(float leftRPM, float rightRPM) {
        float left = getWheelLinearVel(leftRPM);    
        float right = getWheelLinearVel(rightRPM);

        float returned = (left - right) / drivetrain.trackWidth;
        return returned;
    }

};
