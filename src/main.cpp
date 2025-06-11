#include "main.h"
#include "drivecode/ladybrown.hpp"
#include "drivecode/intake.hpp"
#include "drivecode/pistons.hpp"
#include "drivecode/loops.hpp"
#include "globals.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "kiwirun/includes.hpp"
#include "pros/screen.h"

void initialize() {
    chassis.calibrate();
    chassis.setPose(0, 0, 0);

    //screenInit();
    intakeInit();
    //pistonInit();
    //ladyBrownInit();
}

void disabled() {}

void competition_initialize() {}

//temp, move to separate file later
void redRingSide() { //should work
    sortState = 2;

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    ladyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    //alliance
    // chassis.moveToPose(-7, 3.4, -50, 1500);
    chassis.turnToHeading(-50, 500);
    chassis.moveToPoint(-7, 3.4, 1000); //-6.3, 2.7
    chassis.waitUntilDone();
    pidActive = true;
    lbTarget = 205;

    pros::delay(500);//750
    // pidActive = true;
    // lbTarget = 2;

    //mogo
    // chassis.moveToPoint(5.3, -7.2, 750, {.forwards = false});
    // chassis.turnToHeading(0, 500);
    chassis.moveToPose(4, -24, 0, 2400, {.forwards = false, .minSpeed = 60});
    pros::delay(250);
    pidActive = true;
    lbTarget = 2;
    chassis.moveToPoint(4, -32, 1000, {.forwards = false, .maxSpeed = 40});
    // chassis.moveToPose(6.9, -30.5, 0, 2400, {.forwards = false, .lead = 0.2, .minSpeed = 50}); //try changing to move to pose
    chassis.waitUntilDone();
    pros::delay(100);
    clampState = 1;
    pros::delay(100);
    intakeState = 2;

    //middle rings
    chassis.turnToHeading(132.1,3000, {.minSpeed = 70});
    chassis.waitUntilDone();
    intakeState = 1;
    chassis.moveToPose(35, -41, 90, 1000, {.minSpeed = 70});
    chassis.moveToPoint(41.5, -41, 1000, {.maxSpeed = 60}); //TODO: prev 43.25
    chassis.waitUntilDone();
    pros::delay(250);

    // chassis.moveToPose(15.04, -31.95, 130.5, 2000, {.forwards = false});
    // chassis.turnToHeading(61, 1000);
    // chassis.waitUntilDone();
    // pros::delay(200);
    // // chassis.moveToPoint(29.16, -26.94, 1000);
    // chassis.moveToPoint(30, -28, 1000);
    // chassis.moveToPose(51.23, 5.85, 41.8, 3000);

    //1 ring
    chassis.moveToPose(7.6, -22.7, 138, 1500, {.forwards = false, .minSpeed = 90});
    chassis.waitUntilDone();
    chassis.turnToHeading(84, 500);
    chassis.moveToPoint(35, -23.6, 1000, {.minSpeed = 100});
    chassis.waitUntilDone();
    pros::delay(1000);
    
    // //ring stack
    // chassis.turnToHeading(0, 500);
    // chassis.moveToPoint(38.5, -3, 1000);
    // chass

    // return;

    //ladder
    chassis.turnToHeading(-90, 1000);
    chassis.moveToPoint(-17, -23.6, 5000, {.maxSpeed = 80});
    

    // //fifth ring
    // chassis.moveToPoint(32.9, -4.3, 1000, {.forwards = false, .maxSpeed = 50});
    // chassis.turnToHeading(230, 1200);

    // chassis.moveToPoint(0, -27.3, 3000);
    // chassis.waitUntilDone();

    return;
}

void blueRingSide() { //should work
    sortState = 1;

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    ladyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    //alliance
   
    chassis.turnToHeading(50, 500);
    chassis.moveToPoint(7, 3.4, 1000); //-6.3, 2.7
    chassis.waitUntilDone();
    pidActive = true;
    lbTarget = 195;

    pros::delay(500);//750
    // pidActive = true;
    // lbTarget = 2;

    //mogo
   
    chassis.moveToPose(-4.5, -24, 0, 2400, {.forwards = false,  .minSpeed = 40});
    pros::delay(250);
    pidActive = true;
    lbTarget = 2;
    chassis.moveToPoint(-4.5, -34, 1000, {.forwards = false, .maxSpeed = 40});
    
    chassis.waitUntilDone();
    pros::delay(100);
    clampState = 1;
    pros::delay(100);
    intakeState = 2;

    // //middle rings
    chassis.turnToHeading(-150.7,3000, {.minSpeed = 70});
    chassis.waitUntilDone();
    intakeState = 1;
    chassis.moveToPose(-35, -49, -90, 1000, {.minSpeed = 70});
    chassis.moveToPoint(-39.5, -49, 1000, {.maxSpeed = 60}); //TODO: prev 43.25
    chassis.waitUntilDone();
    pros::delay(500);

   

    // // //1 ring
    chassis.moveToPose(-1, -30.2, -136, 1500, {.forwards = false, .minSpeed = 90});
    chassis.waitUntilDone();
    chassis.turnToHeading(-85, 900);
    chassis.moveToPoint(-23.9, -36, 2000);
    chassis.waitUntilDone();
    pros::delay(2000);
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(16.1, -39.1 , 5000, {.maxSpeed = 80});
    

    return;
}

void blueMogo() {
    
    //going to clamp 
    chassis.moveToPoint(0, -31, 1000, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(200 ); 

    //clamp onto the goal 
    clampState = 1;
    
    pros::delay(250 ); 



    //turn to face ring stack 
    sortState = 1;
    chassis.turnToHeading(90, 1000);
    chassis.waitUntilDone();
    intakeState = 1;

    //intake the ring # 1 

    chassis.moveToPoint(23,-31, 1000);

    //turn and go to ring # 2 

    chassis.turnToHeading(0, 1000);

    chassis.moveToPoint(23, -8, 2000);

    pros::delay(500);

    //turn to face corner and go

    chassis.turnToHeading(45, 1000);
    chassis.waitUntilDone();
    //intakeState = 2;

    chassis.moveToPoint(38, 13, 2000); //9
    chassis.waitUntilDone();
    //intakeState = 1;

    pros::delay(1500);

    // //option numero uno: get ring by alliance and touch ladder
    // //go to rings by alliance stake
    // chassis.moveToPoint(16.5, 1, 1000, {.forwards = false});
    // chassis.turnToHeading(-90, 1000);
    // chassis.moveToPoint(-10, -7, 1000, {.maxSpeed = 60});
    // chassis.moveToPoint(-30, -7, 2000, {.minSpeed = 80});
    // chassis.waitUntilDone();
    // pros::delay(1000);

    // //ladder
    // chassis.turnToHeading(170, 500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
    // chassis.moveToPoint(-22.22, -20, 1000);
    // //end of option 1


    //option number dos: straight to ladder
    chassis.moveToPoint(16.5, 1, 1000, {.forwards = false});
    chassis.moveToPose(-12.6, -26.7, 227, 2000, {.minSpeed = 80});
    chassis.waitUntilDone();
    pidActive = true;
    lbTarget = 165;
    //end of option 2


    // //option number tres: drop and go to center mogo
    // chassis.moveToPoint(16.5, 1, 1000, {.forwards = false});
    // pros::delay(1000);
    // chassis.turnToHeading(-136.5, 500);
    // chassis.waitUntilDone();
    // clampState = 0;
    // chassis.moveToPose(19.8, -23, 0, 2000, {.forwards = false});
    // //end of option 3




    //go back to the positio0n before corner
    
    //chassis.moveToPoint(16,-4, 2000, {.forwards = false});
    // //ring 2 (wrong color)
    // intakeState = 2;
    // chassis.moveToPoint(29.7, 13.3, 2000);
    // chassis.waitUntilDone();
    // intakeState = 1;
    // pros::delay(1500);

    // //ring 3
    // chassis.moveToPoint(16,-4, 2000, {.forwards = false});
    // intakeState = 2;
    // chassis.moveToPoint(29.7, 13.3, 2000);
    // chassis.waitUntilDone();
    // intakeState = 1;
    // pros::delay(1500);



    /* extra stuff if time permits


    //turn to face rings in middle ladder

    chassis.turnToHeading(226.5, 1000);

    chassis.moveToPose(-14.3, -35, 200, 3000, {.lead = 0.3, .minSpeed = 80});

    chassis.waitUntilDone();

    leftDoinkerState = 1;

    pros::delay(100);

    return;

    //move backwards

    chassis.moveToPoint(-11, -26.8, 2000, {.forwards = false});

    //turn to intake the ring

    chassis.turnToHeading(-155, 1000);

    return;
*/



}

void redMogo() { //YAY WORKS
    
    //going to clamp 
    chassis.moveToPoint(0, -29, 1000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    pros::delay(200 ); 

    //clamp onto the goal 
    clampState = 1;
    
    pros::delay(100); 



    //turn to face ring stack 
    sortState = 2;
    chassis.turnToHeading(-90, 700);
    chassis.waitUntilDone();
    intakeState = 1;

    //intake the ring # 1 

    chassis.moveToPoint(-22,-34, 1000);
    

    //turn and go to ring # 2 
    pros::delay(1000);
    chassis.turnToHeading(0, 1000);
    chassis.waitUntilDone();
    pros::delay(250);

    chassis.moveToPoint(-21, -14, 2000);

    pros::delay(200);

    //turn to face corner and go

    chassis.turnToHeading(-45, 1000);
    chassis.waitUntilDone();
    pros::delay(250);
    //intakeState = 2;
    chassis.moveToPoint(-45.8, 9.02, 2000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(400);
    //intakeState = 1;
    pros::delay(1000);


    // //option number uno: get alliance ring and go to ladder
    // chassis.moveToPoint(-23, -10, 2000, {.forwards = false});
    // chassis.turnToHeading(90, 1000);

    // chassis.moveToPoint(8, -7, 2000);
    // chassis.moveToPoint(30, -7, 2000, {.maxSpeed = 60});
    // pros::delay(300);

    // //ladder
    // chassis.turnToHeading(200, 1000);
    // chassis.moveToPoint(23, -23, 2000, {.maxSpeed = 60});
    // //end of option 1


    //option number dos: straight to ladder
    //touches ladder in center for some reason
    chassis.moveToPoint(-23, -10, 2000, {.forwards = false});
    chassis.moveToPoint(4.8, -47.4, 5000, {.forwards = false, .minSpeed = 80});
    chassis.waitUntilDone();
    pidActive = true;
    lbTarget = 35;
    //end of option 2


    // //option number tres: drop and go to center mogo
    // chassis.moveToPoint(-23, -10, 2000, {.forwards = false});
    // pros::delay(1000);
    // chassis.turnToHeading(136.5, 500);
    // chassis.waitUntilDone();
    // clampState = 0;
    // chassis.moveToPose(-25, -30, 0, 2000, {.forwards = false});
    // //end of option 3

}

void blueMogoRush() { 
    //goal rush
    intakeState = 3;
    secondStage.move_voltage(0);
    chassis.moveToPoint(0, 37, 1500);
    chassis.waitUntil(33);
    rightDoinkerState = 1;

    pros::delay(250);

    //pull back and drop mogo
    chassis.moveToPoint(0, 18, 1000, {.forwards = false});
    chassis.waitUntil(20);
    rightDoinkerState = 0;

    //get mogo
    intakeState = 0;
    chassis.turnToHeading(-93, 500);
    chassis.moveToPoint(13.6, 14.3, 1000, {.forwards = false});
    chassis.waitUntilDone();
    clampState = 1;
    pros::delay(200);
    intakeState = 1;
    pros::delay(1000);
    clampState = 0;
    pros::delay(100);

    //go to other mogo
    intakeState = 2;
    chassis.moveToPoint(1.9, 13.5, 1000);
    chassis.turnToHeading(-177, 500);
    chassis.moveToPoint(5.8, 23.7, 1000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    clampState = 1;
    pros::delay(100);

    //move to corner
    sortState = 1;
    intakeState = 1;
    chassis.turnToHeading(-144.4, 750);
    chassis.moveToPoint(-9.3, -1.7, 750);
    chassis.waitUntilDone();
    pros::delay(900); //TODO: decrease

    chassis.turnToHeading(-109.6, 750);
    chassis.waitUntilDone();
    chassis.moveToPoint(-29.1, -10.6, 2000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(600);

    //move to ladder
    chassis.moveToPoint(0.8, -0.1, 1000, {.forwards = false, .maxSpeed = 40});
    chassis.waitUntil(15);
    clampState = 0;

    chassis.turnToHeading(35, 600);
    pros::delay(100);
    chassis.moveToPoint(13.5, 18.7, 1000);
    chassis.turnToHeading(71.1, 500);
    pros::delay(1000);
    pidActive = true;
    lbTarget = 165;

    // chassis.moveToPoint(-24.3, -11.1, 750, {.minSpeed = 90});
    // pros::delay(1000);
    // intakeState = 1;

    return;

    // chassis.moveToPose(-25.1, -12.4, -116.5, 10000, {.minSpeed = 70});
    // pros::delay(400);
    // intakeState = 1;

    //corner tomfoolery
    // chassis.turnToHeading(-142.7, 500);
    // chassis.moveToPoint(-8.85, -0.9, 500);
    // chassis.turnToHeading(-119.5, 500);
    // chassis.moveToPoint(-25.05, -11.03, 500, {.minSpeed = 90});
    // pros::delay(400);
    // intakeState = 1;
}

void redMogoRush() {

    
    //goal rush
    intakeState = 3;
    secondStage.move_voltage(0);
    chassis.moveToPoint(0, 37, 1500, {.minSpeed = 20});
    chassis.waitUntil(33);
    leftDoinkerState = 1;

    pros::delay(250);

    //pull back and drop mogo
    chassis.moveToPoint(0, 18, 1000, {.forwards = false, .minSpeed = 20});
    chassis.waitUntil(20);
    leftDoinkerState = 0;

    //get mogo
    intakeState = 0;
    chassis.turnToHeading(103.3, 500);
    chassis.moveToPoint(-15.3, 22.1, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    clampState = 1;
    pros::delay(200);
    intakeState = 1;
    pros::delay(1000);
    clampState = 0;
    pros::delay(100);

    //go to other mogo
    intakeState = 2;
    chassis.moveToPoint(-1.8, 19.3, 1000);
    chassis.turnToHeading(169.3, 500);
    chassis.moveToPoint(-3.8, 32.9, 1000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    clampState = 1;
    pros::delay(100);

    //move to corner
    sortState = 2;
    intakeState = 1;
    chassis.turnToHeading(145.3, 750);
    chassis.moveToPoint(15, 7.3, 750);
    chassis.waitUntilDone();
    pros::delay(200); //TODO: decrease





    chassis.turnToHeading(111.6, 700);


    chassis.moveToPoint(32.4, 1.1, 1000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(1000);


    // //move to ladder
    chassis.moveToPoint(17.7, 3.1, 1000, {.forwards = false});
    chassis.waitUntilDone();

    chassis.turnToHeading(-50.7, 600);
    chassis.waitUntilDone();
    clampState = 0;
    pros::delay(100);
    chassis.moveToPoint(-24.4, 31.8, 4000);
    chassis.waitUntilDone();

     pidActive = true;
     lbTarget = 140;
     
}

void redSigSoloWP(){
    sortState = 2;

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    ladyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    //alliance
    // chassis.moveToPose(-7, 3.4, -50, 1500);
    chassis.turnToHeading(-50, 500);
    chassis.moveToPoint(-7, 3.4, 1000); //-6.3, 2.7
    chassis.waitUntilDone();
    pidActive = true;
    lbTarget = 195;

    pros::delay(500);//750
    pidActive = true;
    lbTarget = 2;

    //mogo
    // chassis.moveToPoint(5.3, -7.2, 750, {.forwards = false});
    // chassis.turnToHeading(0, 500);
    chassis.moveToPose(4, -24, 0, 2400, {.forwards = false, .minSpeed = 40});
    chassis.moveToPoint(4, -32, 1000, {.forwards = false, .maxSpeed = 40});
    // chassis.moveToPose(6.9, -30.5, 0, 2400, {.forwards = false, .lead = 0.2, .minSpeed = 50}); //try changing to move to pose
    chassis.waitUntilDone();
    pros::delay(100);
    clampState = 1;
    pros::delay(100);
    intakeState = 1;

    //first ring
    chassis.turnToHeading(90, 500);
    chassis.moveToPoint(24.8, -28.9, 1000);

    //go to alliance ring and unclamp mogo
    chassis.moveToPoint(15, -2, 2000);
    chassis.turnToHeading(-85, 500);
    chassis.waitUntilDone();
    clampState = 0;
    chassis.moveToPoint(-15, 1, 2000);
    chassis.moveToPoint(-52.6, 1, 3000, {.maxSpeed = 60});

    //throw blue, keep red
    color = 2;
    pros::Task swpTask(swpAuton, "swp");
    pros::delay(250);
    chassis.turnToHeading(-3, 500);
    chassis.moveToPoint(-50, -23, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    clampState = 1;
    pros::delay(100);

    //turn to last ring
    intakeState = 1;
    chassis.turnToHeading(-87, 500);
    chassis.moveToPoint(-80.1, -22.7, 1000);

    //ladder
    chassis.turnToHeading(-80, 500);
    chassis.moveToPoint(-41, -34.8, 1000, {.forwards = false});
    chassis.waitUntilDone();
    pidActive = true;
    lbTarget = 28;
}

void blueSigSoloWP(){
    sortState = 1;

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    ladyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    //alliance
    // chassis.moveToPose(-7, 3.4, -50, 1500);
    chassis.turnToHeading(50, 500);
    chassis.moveToPoint(7, 3.4, 1000); //-6.3, 2.7
    chassis.waitUntilDone();
    pidActive = true;
    lbTarget = 195;

    pros::delay(500);//750
    pidActive = true;
    lbTarget = 2;

    //mogo
    // chassis.moveToPoint(5.3, -7.2, 750, {.forwards = false});
    // chassis.turnToHeading(0, 500);
    chassis.moveToPose(-4, -24, 0, 2400, {.forwards = false, .minSpeed = 40});
    chassis.moveToPoint(-4, -32, 1000, {.forwards = false, .maxSpeed = 40});
    // chassis.moveToPose(6.9, -30.5, 0, 2400, {.forwards = false, .lead = 0.2, .minSpeed = 50}); //try changing to move to pose
    chassis.waitUntilDone();
    pros::delay(100);
    clampState = 1;
    pros::delay(100);
    intakeState = 1;

    //first ring
    chassis.turnToHeading(-90, 500);
    chassis.moveToPoint(-24.8, -28.9, 1000);

    //go to alliance ring and unclamp mogo
    chassis.moveToPoint(-15, -2, 2000);
    chassis.turnToHeading(85, 500);
    chassis.waitUntilDone();
    clampState = 0;
    chassis.moveToPoint(15, 1, 2000);
    chassis.moveToPoint(52.6, 1, 3000, {.maxSpeed = 60});

    //throw blue, keep red
    color = 1;
    pros::Task swpTask(swpAuton, "swp");
    pros::delay(250);
    chassis.turnToHeading(3, 500);
    chassis.moveToPoint(50, -25, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    clampState = 1;
    pros::delay(100);

    //turn to last ring
    intakeState = 1;
    chassis.turnToHeading(87, 500);
    chassis.moveToPoint(80.1, -26.2, 1000);

    //ladder
    chassis.turnToHeading(80, 500);
    chassis.moveToPoint(41, -34.8, 1000, {.forwards = false});
    chassis.waitUntilDone();
    pidActive = true;
    lbTarget = 28;
}

void autonomous() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    // kiwi::Path filePath(config, "/usd/path.json");
    // filePath.follow();
  
    //redRingSide();
    blueRingSide();
    
    //blueMogo(); 
    
    //redMogo();
    //blueMogoRush();
    //redMogoRush();
    //redSigSoloWP();
    //blueSigSoloWP();
}

void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    bool tank = true;

    while(true) {

        // TANK--DO CONTROL K THEN CONTROL C TO COMMENT
        if(tank) {
            int left = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int right = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
            leftMotors.move_velocity(left);
            rightMotors.move_velocity(right);
        } else {
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
        }
        

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            tank = !tank;
        }

        updateIntake();
        //updatePistons();
        //updateLB();

        pros::delay(10);
    }

    //kiwiRunControl();

    //matchControl();
    

    //autonControl();

    // firstPath.follow();
}
