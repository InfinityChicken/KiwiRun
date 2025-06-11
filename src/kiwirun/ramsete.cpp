#include <cmath>
#include "kiwirun/includes.hpp"
#include "globals.hpp"

std::string debugLine;

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

void Path::updateSubsystems(int index) { //TODO: test subsystem functionality
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

    lemlib::Pose target = this->pathRecordings[index]; // target pose fetched

    float targetX = toMeters(target.x); //in meters
    float targetY = toMeters(target.y); //in meters
    float targetTheta = target.theta;

    float linearVelTarget = toMeters(this->velRecordings[index][0]); // target velocities fetched
    float angularVelTarget = this->velRecordings[index][1];

    lemlib::Pose pose = this->config.chassis.getPose(true);

    float errorLateral = findLateralError(targetX, targetY); // lateral or crosstrack error calculated in meters
    float errorLongitudinal = findLongitudinalError(targetX, targetY); // longitudinal or front/back error calculated in meters
    float errorTheta = fmod((targetTheta - pose.theta), M_PI*2); // angular error calculated

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

    // leftBack.move_velocity(leftRPMCommand / 3); //* sketch 55w implementation lol but it'll work
    // rightBack.move_velocity(rightRPMCommand / 3);

    debugLine.append("current x inches: " + std::to_string(pose.x) + "\n"); //*DEBUG LINES FOR CURRENT XYTHETA
    debugLine.append("current y inches: " + std::to_string(pose.y) + "\n");

    debugLine.append("current x meters: " + std::to_string(toMeters(pose.x)) + "\n"); //*DEBUG LINES FOR CURRENT XYTHETA
    debugLine.append("current y meters: " + std::to_string(toMeters(pose.y)) + "\n");

    debugLine.append("current theta rad: " + std::to_string(pose.theta) + "\n");

    debugLine.append("target x inches: " + std::to_string(toInches(targetX)) + "\n"); //*DEBUG LINES FOR TARGET XYTHETA
    debugLine.append("target y inches: " + std::to_string(toInches(targetY)) + "\n");

    debugLine.append("target x meters: " + std::to_string(targetX) + "\n"); //*DEBUG LINES FOR TARGET XYTHETA
    debugLine.append("target y meters: " + std::to_string(targetY) + "\n");

    debugLine.append("target theta rad: " + std::to_string(targetTheta) + "\n");

    debugLine.append("error lateral inches: " + std::to_string(toInches(errorLateral)) + "\n"); //*DEBUG LINES FOR ERRORS
    debugLine.append("error longitudinal inches: " + std::to_string(toInches(errorLongitudinal)) + "\n");

    debugLine.append("error lateral meters: " + std::to_string(errorLateral) + "\n"); //*DEBUG LINES FOR ERRORS
    debugLine.append("error longitudinal meters: " + std::to_string(errorLongitudinal) + "\n");

    debugLine.append("error theta: " + std::to_string(errorTheta) + "\n");

    debugLine.append("current left rpm: " + std::to_string(this->config.leftMotors.get_actual_velocity()) + "\n"); //*DEBUG LINES FOR CURRENT RPMS
    debugLine.append("current right rpm: " + std::to_string(this->config.rightMotors.get_actual_velocity()) + "\n");

    debugLine.append("gain: " + std::to_string(gain) + "\n"); //*DEBUG LINES FOR GAIN

    debugLine.append("linear vel target meters: " + std::to_string(linearVelTarget) + "\n"); //*DEBUG LINES FOR VEL TARGETS
    debugLine.append("linear vel target inches: " + std::to_string(toInches(linearVelTarget)) + "\n"); //*DEBUG LINES FOR VEL TARGETS
    debugLine.append("angular vel target: " + std::to_string(angularVelTarget) + "\n");

    debugLine.append("linear vel command inches: " + std::to_string(linearVelCommand) + "\n");//*DEBUG LINES FOR VEL COMMANDS
    debugLine.append("angular vel command: " + std::to_string(angularVelCommand) + "\n");
    debugLine.append("tangential vel command inches: " + std::to_string(tangentialVelCommand) + "\n");

    debugLine.append("left rpm command: " + std::to_string(leftRPMCommand) + "\n"); //*DEBUG LINE FOR LEFT AND RIGHT RPM
    debugLine.append("right rpm command: " + std::to_string(rightRPMCommand) + "\n\n\n"); //*LAST DEBUG LINE
}

void Path::follow() {
    int index = 0;

    this->fetchData(); //fetch data

    std::ofstream debugFile("/usd/debug.txt", std::ios::out | std::ios::trunc);

    while (true) {

        debugLine.append("NEW TICK\n\n");

        index = findClosestPoint(this->config.chassis.getPose(), index);

        debugLine.append("closest index: " + std::to_string(index) + "\n\n");

        if (index == -1) {
            break;
        }

        ramseteStep(index);
        updateSubsystems(index);

        debugFile << debugLine;
        debugLine.clear();

        pros::delay(10);

    }

    return;

}

}
