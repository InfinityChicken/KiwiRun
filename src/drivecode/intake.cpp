#include "drivecode/intake.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include <queue>

int color = 0;

bool l1Pressed = false;
bool l2Pressed = false;
bool downPressed = false;
bool upPressed = false;

//color sort states
bool throwNext = false;
bool ringDetected = false;
bool distDetected = false;
int sortState;

float intakeState = 0;

std::queue<bool> throwRings;
bool throwPushed = false;

void intakeInit() {
    firstStage.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    secondStage.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    optical.set_led_pwm(100);

    pros::Task intakeTask(runIntake, "intake");
    pros::Task colorTask(colorSort, "color sort");
}

void updateIntake() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        if (!l1Pressed) {
            l1Pressed = true;
            if (intakeState == 0 || intakeState == 2 || intakeState == 3) {
                intakeState = 1;
            } else if (intakeState == 1) {
                intakeState = 0;
            }
        }
    } else {
        l1Pressed = false;
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        if (!l2Pressed) {
            l2Pressed = true;
            if (intakeState == 0 || intakeState == 1 || intakeState == 3) {
                intakeState = 2;
            } else if (intakeState == 2) {
                intakeState = 0;
            }
        }
    } else {
        l2Pressed = false;
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
        if (!downPressed) {
            downPressed = true;
            if (intakeState == 0 || intakeState == 1 || intakeState == 2) {
                intakeState = 3;
            } else if (intakeState == 3) {
                intakeState = 0;
            }
        }
    } else {
        downPressed = false;
    }

    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
        if(!upPressed) {
            upPressed = true;
            sortState++;
            if(sortState == 3) {
                sortState = 0;
            }
        }
    } else {
        upPressed = false;
    }
}

void runIntake() {
    while(true) {
        if (intakeState == 0) {
            firstStage.move_voltage(0);
            secondStage.move_voltage(0);
        } else if (intakeState == 1) {
            firstStage.move_voltage(12000);
            secondStage.move_voltage(12000);
        } else if (intakeState == 2) {
            firstStage.move_voltage(-12000);
            secondStage.move_voltage(-12000);
            throwNext = false;
        } else if(intakeState == 3) {
            firstStage.move_voltage(12000);
            secondStage.move_voltage(0);
        }

        pros::delay(10);
    }
}

void colorSort() {
    while(true) {
        //no throw logic
        if(sortState == 0) {
            controller.set_text(0, 0, "no sort   ");
            //throwNext = false;
            ringDetected = false;
        }

        //throw red logic
        else if(sortState == 1) {
            controller.set_text(0, 0, "sort red   ");

            if(0 < optical.get_hue() && optical.get_hue() < 20) { //if a newly detected ring is red,
                if(!ringDetected) { //and a ring hasn't been detected yet,
                    throwRings.push(true);
                    //throwNext = true; //throw the next ring
                    ringDetected = true;

                    std::cout<<"I SAW RED "<<"\n";
                }
            } else if(100 < optical.get_hue() && optical.get_hue() < 240) {
                if(!ringDetected) { //and a ring hasn't been detected yet,
                    throwRings.push(false);
                    //throwNext = true; //throw the next ring
                    ringDetected = true;

                    std::cout<<"I SAW BLUE "<<"\n";
                }
            } else {
                ringDetected = false;
            }
        }

        //throw blue logic
        else if(sortState == 2) {
            controller.set_text(0, 0, "sort blue   ");

            if(100 < optical.get_hue() && optical.get_hue() < 240) { //if a newly detected ring is red,
                if(!ringDetected) { //and a ring hasn't been detected yet
                    throwRings.push(true);
                    //throwNext = true; //throw the next ring
                    ringDetected = true;

                    std::cout<<"I SAW BLUE"<<"\n";
                }
            } else if(0 < optical.get_hue() && optical.get_hue() < 20) {
                if(!ringDetected) { //and a ring hasn't been detected yet,
                    throwRings.push(false);
                    //throwNext = true; //throw the next ring
                    ringDetected = true;

                    std::cout<<"I SAW RED "<<"\n";
                }
            } else {
                ringDetected = false;
            }

        }

        //actual throw logic
        if(sortState != 0 && distance.get() < 30) { //if throw next and a ring is about to be scored,
            if(!distDetected) {
                if(throwRings.front()) {
                    float prevIntake = intakeState;

                    pros::delay(140);

                    intakeState = 2; //throw
                    pros::delay(100);
                    intakeState = prevIntake;

                    std::cout<<"I THREW IT"<<"\n";

                    //throwNext = false;
                }
                if(!throwRings.empty())
                    throwRings.pop();
                distDetected = true;
            }
        } else {
            distDetected = false;
        }
        

        pros::delay(10);
    }
}

void swpAuton() {
    bool colorFound = false;
    while(!colorFound) {
        if(color == 1) { // blue
            if(100 < optical.get_hue() && optical.get_hue() < 240) {
                intakeState = 0;
                colorFound = true;
            }
        }
        if(color == 2) { // red
            if(0 < optical.get_hue() && optical.get_hue() < 20) {
                intakeState = 0;
                colorFound = true;
            }
        }
    }
    return;
}