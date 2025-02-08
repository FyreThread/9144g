#pragma once

#include "pros/motors.hpp"

namespace my_robot {

void handleMotorStall(pros::Motor &motor, int forwardPower, int reversePower, int reverseTimeMs);
void stopIntakeOnSecondPress();
void stopIntakeOnFirstPress();
void intakeAutomation();
void wallThings(int number);
void calibrateWallStake();

}  // namespace my_robot