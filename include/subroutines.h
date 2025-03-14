#pragma once

#include "pros/motors.hpp"

namespace my_robot {

void handleMotorStall(pros::Motor &motor, int forwardPower, int reversePower, int reverseTimeMs);
void stopIntakeOnSecondPress();
void stopIntakeOnFirstPress();
void intakeAutomation();
void wallThings(int number, bool calibrate);
void calibrateWallStake();
void colorSort(int waitTimeMs, int stopTimeMs, bool red_Side);
void stopIntakeOnRedRing();
}  // namespace my_robot