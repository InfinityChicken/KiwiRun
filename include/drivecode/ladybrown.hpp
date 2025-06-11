#ifndef LADYBROWN_HPP
#define LADYBROWN_HPP

#include "main.h"
#include "globals.hpp"
#include "pros/rotation.hpp"

inline pros::Motor ladyBrown(13, pros::MotorGearset::red);
inline pros::Rotation lbRotation(7);

void ladyBrownInit();
void updateLB();
void runLB();

extern bool pidActive;
extern float lbTarget;

#endif