#include "drivecode/ladybrown.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

bool pidActive = false;
float lbTarget = 0;
int descoreState = 0;
int prevDescoreState = 0;
bool pidToggle = false;
bool rightPressed = false;

void ladyBrownInit() {
    ladyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    ladyBrown.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

    pros::Task lbTask(runLB, "ladybrown");
}

void updateLB() {
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        descoreState = prevDescoreState;
        pidActive = true;
        lbTarget = 27;
    }

    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        descoreState = prevDescoreState;
        pidActive = true;
        lbTarget = 4;
    }

    // if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    //     pidActive = true;
    //     lbTarget = 150;
    // }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        if (!rightPressed) {
            rightPressed = true;
            prevDescoreState = descoreState;
            if (descoreState == 0) {
                descoreState = 1;
                pidActive = true;
                lbTarget = 80;
            } else if (descoreState == 1) {
                descoreState = 2;
                pidActive = true;
                lbTarget = 168;
            } else if (descoreState == 2) {
                descoreState = 3;
                pidActive = true;
                lbTarget = 172;
            } else if (descoreState == 3) {
                descoreState = 0;
                pidActive = true;
                lbTarget = 184;
            }
        }
    } else {
        rightPressed = false;
    }

}

void runLB() {

    const float kP = 5.5;
    const float kD = 4.7;
    float prevError = 0;

    int count = 1;

    while (true) {
        //std::cout<<"pos: "<<lbRotation.get_angle()/100<<"\n";

        if(!pidActive) { //if pid inactive, enable right stick control 
            float command = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) / 127.0 * 100.0; 
            ladyBrown.move_velocity(command);

            pros::delay(10);

        } else { //if pid active, run pid

            float pos = lbRotation.get_angle() / 100.0;

            if(pos > 350) {
                pos = 0;
            }

            //std::cout<<pos<<"\n";

            float error = lbTarget - pos;
            //std::cout<<"error: "<<error<<"\n";

            float derivative = prevError - error;
            //std::cout<<"derivative: "<<derivative<<"\n";

            float pTerm = error * kP;
            float dTerm = derivative * kD;

            float command = pTerm + dTerm; //calculate p and d terms

            if(command > 100) { //clamp command
                command = 100;
            } else if (command < -100) {
                command = -100;
            }

            //std::cout<<"command: "<<command<<"\n\n";

            if(std::abs(error) < 1) { //pid timeout to enable manual control: .25 seconds within 1deg to target
                count++;

                if(count == 10) {
                    pidActive = false;
                    prevError = 0;
                }

            } else {
                count = 1;
            }

            ladyBrown.move_velocity(command); //move lb
        }

        //std::cout<<lbRotation.get_position() / 100 <<"\n";

        pros::delay(10);
    }
}