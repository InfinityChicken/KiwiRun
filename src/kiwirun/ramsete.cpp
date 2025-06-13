#include <cmath>
#include "kiwirun/includes.hpp"
#include "globals.hpp"

nlohmann::json debugData;

namespace kiwi {

float Path::findLateralError(float targetX, float targetY) { // lateral error

    lemlib::Pose currentPose = this->config.chassis.getPose(true);

    float deltaX = targetX - toMeters(currentPose.x);
    float deltaY = targetY - toMeters(currentPose.y);

    return deltaX * std::cos(currentPose.theta) + deltaY * std::sin(currentPose.theta);
}

float Path::findLongitudinalError(float targetX, float targetY) { // longitudinal error

    lemlib::Pose currentPose = this->config.chassis.getPose(true);

    float deltaX = targetX - toMeters(currentPose.x);
    float deltaY = targetY - toMeters(currentPose.y);

    return -1 * deltaX * std::sin(currentPose.theta) + deltaY * cos(currentPose.theta);
}

int Path::findClosestPoint(lemlib::Pose pose, int prevIndex) {
    int closestIndex = prevIndex;
    float closestDist = infinity();

    if(closestIndex + 1 >= pathRecordings.size()) {
        return -1;
    } //end of path reached

    for (int i = closestIndex + 1; i < this->pathRecordings.size(); i++) {
        // loop starting at ONE FORWARD THE PREVIOUSLY CLOSEST POINT! SKIP / STOP TOLERANCE!
        // and ending at the end of the path

        const float dist = std::abs(pose.distance(pathRecordings[i])); // distance to pose

        if (dist < closestDist) { // if tested point is closer, record it and move on
            closestDist = dist;
            closestIndex = i;
        } else if (dist > closestDist) {
            prevIndex = closestIndex;
            return closestIndex; // if distance increases because you're past the closest point, return the closest pose
        }
    }

    return -1; // you're screwed OR end of path reached
}

void Path::updateSubsystems(int index) {
    for (int i = 0; i < this->subsysRecordings[index].size(); i++) {
        this->config.subsysStates[i] = this->subsysRecordings[index][i]; // loop through all subsystem states and update
    }
}

float Path::toRPM(float linearVel) {
    float corrected = linearVel / (M_PI * this->config.drivetrain.wheelDiameter) * 60 * this->config.driven / this->config.driving;
    return corrected;
}

float Path::toMeters(float inchMeasurement) {
    return inchMeasurement / 39.3701; 
}

float Path::toInches(float meterMeasurement) {
    return meterMeasurement * 39.3701;
}

void Path::ramseteStep(int index) {

    float beta = this->config.beta; // beta and zeta values fetched
    float zeta = this->config.zeta;

    lemlib::Pose target = this->pathRecordings[index]; // target pose fetched, in inches + rad

    float targetX = toMeters(target.x); //x in meters
    float targetY = toMeters(target.y); //y in meters
    float targetTheta = target.theta;

    float linearVelTarget = toMeters(this->velRecordings[index][0]); // target velocities fetched
    float angularVelTarget = this->velRecordings[index][1];

    lemlib::Pose pose = this->config.chassis.getPose(true);

    float errorLateral = findLateralError(targetX, targetY); // lateral or crosstrack error calculated in meters
    float errorLongitudinal = findLongitudinalError(targetX, targetY); // longitudinal or front/back error calculated in meters

    //corrected such that current pose 0, target somewhere else
    //if targetTheta 350, pose 10, corrected target 340, corrected current 0, true error -20
    //if targetTheta 10, pose 350, corrected target -340, corrected current 0, true error 20
    //if targetTheta 100, pose 10, corrected target 90, corrected current 0, true error 90
    //if targetTheta 300, pose 30, corrected target 270, corrected current 0, true error -90

    float errorTheta = targetTheta - pose.theta; //TODO: check angle

    if (std::abs(errorTheta) > M_PI) {
        if (errorTheta > 0) {
            errorTheta -= 2*M_PI;
        } else {
            errorTheta += 2*M_PI;
        }
    }

    float gain = 2 * zeta * std::sqrt(
        std::pow(angularVelTarget, 2) + (beta * std::pow(linearVelTarget, 2))
    ); // master gain calculated with: sqrt[(angular velocity)^2 + beta * (linear velocity)^2]

    float linearVelCommand =
        (linearVelTarget * std::cos(errorTheta)) 
        + (gain * errorLongitudinal
    ); // linear command calculated with: [linear velocity target * cos(angle error)] + (gain * longitudinal error)

    linearVelCommand = toInches(linearVelCommand); //switch back to inches

    if(std::abs(errorTheta) < 0.01) { //*divide by zero protection
        errorTheta = 0.01;
    }

    float angularVelCommand = 
        angularVelTarget + (gain * errorTheta) + 
        (beta * linearVelTarget * std::sin(errorTheta)
        * errorLateral / errorTheta
    ); // angular command calculated with: beta * linear velocity target * sin(angle error) * lateral error / angle error

    if(angularVelCommand > 3.5) {
        angularVelCommand = 3.5;
    } // angular vel clamp

    float tangentialVelCommand = angularVelCommand * this->config.drivetrain.trackWidth / 2; //convert angular vel command from rad/s to in/s

    float leftRPMCommand = toRPM(linearVelCommand + tangentialVelCommand); //convert in/s of wheels to rpm
    float rightRPMCommand = toRPM(linearVelCommand - tangentialVelCommand);

    float maxRPM = std::max(std::abs(leftRPMCommand), std::abs(rightRPMCommand));

    if(maxRPM > 600) {
        float scale = 600.0 / maxRPM;
        leftRPMCommand *= scale;
        rightRPMCommand *= scale;
    }

    this->config.leftMotors.move_velocity(leftRPMCommand); // velocity sent to motors
    this->config.rightMotors.move_velocity(rightRPMCommand);

    debugData["curr x in"] = pose.x;
    debugData["curr y in"] = pose.y;
    debugData["curr theta rad"] = pose.theta;
    debugData["curr theta deg"] = pose.theta / M_PI * 180;

    debugData["target x in"] = target.x;
    debugData["target y in"] = target.y;
    debugData["target theta rad"] = target.theta;
    debugData["target theta deg"] = target.theta / M_PI * 180;

    debugData["error lat in"] = toInches(errorLateral);
    debugData["error long in"] = toInches(errorLongitudinal);
    debugData["error theta rad"] = errorTheta;
    debugData["error theta deg"] = errorTheta / M_PI * 180;
    
    debugData["gain"] = gain;
    debugData["target linear vel in"] = toInches(linearVelTarget);
    debugData["target angular vel rad"] = angularVelTarget;
    
    debugData["cmd linear vel in"] = linearVelCommand;
    debugData["cmd tangent vel in"] = tangentialVelCommand;
    debugData["cmd angular vel rad"] = angularVelTarget;

    debugData["curr left rpm"] = this->config.leftMotors.get_actual_velocity();
    debugData["curr right rpm"] = this->config.leftMotors.get_actual_velocity();

    debugData["cmd left rpm"] = leftRPMCommand;
    debugData["cmd right rpm"] = rightRPMCommand;
}

void Path::follow() {
    int index = 0;

    this->fetchData(); //fetch data

    std::ofstream debugFile("/usd/debug.txt", std::ios::out | std::ios::trunc);

    while (true) {

        index = findClosestPoint(this->config.chassis.getPose(), index);

        debugData["index"] = index;

        if (index == -1) {
            break;
        }

        ramseteStep(index);
        updateSubsystems(index);

        debugFile << debugData.dump();

        pros::delay(10);
        
    }

    return;

}

}
