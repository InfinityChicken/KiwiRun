#include "kiwirun/includes.hpp"
#include "drivecode/intake.hpp"
#include "drivecode/pistons.hpp"
#include "drivecode/ladybrown.hpp"
#include "kiwirun/path.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "globals.hpp"
#include <functional>

/*
current button binds

left joystick x: steer
left joystick y: throttle

right joystick x: unbound
right joystick y: manual lb control

l1: intake (both stages)
l2: outtake (both stages)

r1: load lb
r2: zero lb

a: stop kiwirun recording
b: doinker right
x: just first stage
y: mogo mech

up: cycle color sort
down: intake state 3 [CHANGE TO DOINKER LEFT]
left:
right: cycle descores

scuff buttons are right, down, b, y
*/

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-1, -8, -20}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({5, 14, 3}, pros::MotorGearset::blue);

pros::Imu imu(15);

pros::Rotation lateralSensor(21);

lemlib::TrackingWheel lateralTracker(&lateralSensor, lemlib::Omniwheel::NEW_2, -0.0095);

lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 11.5625,
                              lemlib::Omniwheel::NEW_275, 450, 2);

lemlib::ControllerSettings linearController(7.7, 0, 2, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angularController(2.1, 0, 10.6, 3, 1, 100, 3, 500, 0);

lemlib::OdomSensors sensors(nullptr, nullptr, &lateralTracker, nullptr, &imu);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, nullptr, nullptr);

std::vector<std::reference_wrapper<float>> subsysStates = {intakeState, clampState, lbTarget};
std::vector<std::string> subsysNames = {"intake", "mogo clamp", "lb target"};

kiwi::Config config(1.7, 0.7, 3, 4, 
    subsysStates,
    subsysNames,
    chassis,
    drivetrain,
    leftMotors,
    rightMotors,
    controller
);