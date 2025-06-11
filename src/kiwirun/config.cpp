#include "kiwirun/includes.hpp"

namespace kiwi {
     Config::Config (float beta, float zeta, int driving, int driven,
         std::vector<std::reference_wrapper<float>> subsysStates,
         std::vector<std::string> subsysNames,
         lemlib::Chassis &chassis,
         lemlib::Drivetrain drivetrain,
         pros::MotorGroup &leftMotors,
         pros::MotorGroup &rightMotors,
         pros::Controller controller):
         beta(beta), zeta(zeta), driving(driving), driven(driven), subsysStates(subsysStates), subsysNames(subsysNames), chassis(chassis),
         drivetrain(drivetrain), leftMotors(leftMotors), rightMotors(rightMotors), controller(controller){}
 }
