#ifndef PISTON_HPP
#define PISTON_HPP

#include "main.h"
#include "globals.hpp"

inline pros::adi::DigitalOut mogoClamp('A');
inline pros::adi::DigitalOut leftDoinker('B');
inline pros::adi::DigitalOut rightDoinker('C');

void pistonInit();
void updatePistons();
void runPistons();

extern float clampState;
extern float leftDoinkerState;
extern float rightDoinkerState;

#endif