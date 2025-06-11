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

void opcontrol() {
    matchControl();
}
