#include "drivecode/loops.hpp"
#include "lemlib/pose.hpp"
#include "pros/misc.h"
#include "globals.hpp"

void screenInit() {
    pros::Task screen_task_actual(screenTask, "screen task");
}

void screenTask() {
    while(true) {
        lemlib::Pose pose = chassis.getPose();

        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "x: %.3f", pose.x);
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "y: %.3f", pose.y);
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "heading: %.3f", pose.theta);
    
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "lb position: %.3f", lbRotation.get_angle() / 100.0);
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "color sensor: %.3f", optical.get_hue());
        pros::screen::print(pros::E_TEXT_MEDIUM, 6, "distance sensor: %.3f", distance.get());
        pros::screen::print(pros::E_TEXT_MEDIUM, 7, "left rpm: %.3f", leftMotors.get_actual_velocity(0));
        pros::screen::print(pros::E_TEXT_MEDIUM, 8, "right rpm: %.3f", rightMotors.get_actual_velocity(0));
        // std::cout<<"x: "<<pose.x<<" y: "<<pose.y<<" theta: "<<pose.theta<<"\n";


        pros::delay(50);
    }
}

void matchControl() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    while(true) {

        int rawThrottle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rawTurn = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        float throttle = rawThrottle;
        float turn = rawTurn * 0.9;

        float leftCommand = (throttle + turn) / 127.0;
        float rightCommand = (throttle - turn) / 127.0;

        if(leftCommand > 1) {
            float fix = 1 / leftCommand;
            leftCommand = 1;
            rightCommand *= fix;
        }
        
        if(rightCommand > 1) {
            float fix = 1 / rightCommand;
            leftCommand *= fix;
            rightCommand = 1;
        }

        leftCommand *= 12000;
        rightCommand *= 12000;

        leftMotors.move_voltage(leftCommand);
        rightMotors.move_voltage(rightCommand);

        updateIntake();
        updatePistons();
        updateLB();

        pros::delay(10);
    }
}

void autonControl() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

    while(true) {
        pidActive = true;

        int rawThrottle = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int rawTurn = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        float throttle = rawThrottle;
        float turn = rawTurn * 0.9;

        float leftCommand = (throttle + turn) / 127.0;
        float rightCommand = (throttle - turn) / 127.0;

        if(leftCommand > 1) {
            float fix = 1 / leftCommand;
            leftCommand = 1;
            rightCommand *= fix;
        }
        
        if(rightCommand > 1) {
            float fix = 1 / rightCommand;
            leftCommand *= fix;
            rightCommand = 1;
        }

        leftCommand *= 12000;
        rightCommand *= 12000;

        leftMotors.move_voltage(leftCommand);
        rightMotors.move_voltage(rightCommand);

        updateIntake();
        updatePistons();
        updateLB();

        pros::delay(10);
    }
}


void kiwiRunControl() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    config.write();

    while(true) {

        int rawThrottle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rawTurn = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        if (std::abs(rawTurn) < 15) {
            rawTurn = 0;
        }
        
        if (std::abs(rawThrottle) < 15) {
            rawThrottle = 0;
        }

        float throttle = rawThrottle;
        float turn = rawTurn * 0.9;

        float leftCommand = (throttle + turn) / 127.0;
        float rightCommand = (throttle - turn) / 127.0;

        if(leftCommand > 1) {
            float fix = 1 / leftCommand;
            leftCommand = 1;
            rightCommand *= fix;
        }
        
        if(rightCommand > 1) {
            float fix = 1 / rightCommand;
            leftCommand *= fix;
            rightCommand = 1;
        }

        leftCommand *= 12000;
        rightCommand *= 12000;

        leftMotors.move_voltage(leftCommand);
        rightMotors.move_voltage(rightCommand);
        

        updateIntake();
        updatePistons();

        pros::delay(10);
    }
}