#include "devices.h"
#include "main.h"  // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "subroutines.h"

using namespace my_robot;
bool red_side;
// AUTONOMOUS ROUTINES

void PIDtune() {
  chassis.setPose(0, 0, 0);
  chassis.turnToHeading(90, 100000);
}

void redSWP() {
  red_side = true;
  doinker.set_value(false);
  const int forwardPower = 127;   // Forward motor power (0-127)
  const int reversePower = -127;  // Reverse motor power (0-127)
  const int reverseTimeMs = 300;  // Reverse duration in ms

  // Start a task to handle motor stall detection
  pros::Task motorTask([&]() {
    handleMotorStall(secondStage, forwardPower, reversePower, reverseTimeMs);
  });
  wall_stake.tare_position();
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(0, 0, 0);

  // Start route
  chassis.moveToPose(-13.25, -28.5, 31, 1500,
                     {.forwards = false});  // Move to first mogo
  chassis.waitUntilDone();
  mogo.set_value(true);  // Clamp mogo
  pros::delay(250);
  intake.move(127);
  chassis.moveToPose(3, -26, 86.5, 1750);
  chassis.waitUntilDone();
  chassis.moveToPose(-25.25, -0.85, -90, 2500);
  chassis.waitUntilDone();
  mogo.set_value(false);
  pros::Task stop(stopIntakeOnSecondPress);
  intake.move(127);
  chassis.moveToPose(-40, -6.85, -90, 1000);
  chassis.waitUntilDone();
  chassis.moveToPose(-61, -26, 27.3, 2200,
                     {.forwards = false});  // Move to second mogo
  chassis.waitUntilDone();
  mogo.set_value(true);
  pros::delay(250);
  intake.move(127);
  chassis.turnToHeading(-90, 500);
  chassis.waitUntilDone();
  chassis.moveToPose(-80, -26, -90, 1750);
  chassis.waitUntilDone();
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  chassis.moveToPose(-45, -26, -270, 3000);
  chassis.waitUntilDone();
  intake.move(0);
}

void blueSWP() {
  red_side = false;
  doinker.set_value(false);
  const int forwardPower = 127;   // Forward motor power (0-127)
  const int reversePower = -127;  // Reverse motor power (0-127)
  const int reverseTimeMs = 300;  // Reverse duration in ms

  // Start a task to handle motor stall detection
  pros::Task motorTask([&]() {
    handleMotorStall(secondStage, forwardPower, reversePower, reverseTimeMs);
  });
  wall_stake.tare_position();
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(0, 0, 0);

  // Start route
  chassis.moveToPose(13.25, -28.5, 329, 1500,
                     {.forwards = false});  // Move to first mogo
  chassis.waitUntilDone();
  mogo.set_value(true);  // Clamp mogo
  pros::delay(250);
  intake.move(127);
  chassis.moveToPose(3, -26, 273.5, 1750);
  chassis.waitUntilDone();
  chassis.moveToPose(25.25, -0.85, 90, 2500);
  chassis.waitUntilDone();
  mogo.set_value(false);
  pros::Task stop(stopIntakeOnSecondPress);
  intake.move(127);
  chassis.moveToPose(40, -6.85, 90, 1000);
  chassis.waitUntilDone();
  chassis.moveToPose(61, -26, 332.7, 2200,
                     {.forwards = false});  // Move to second mogo
  chassis.waitUntilDone();
  mogo.set_value(true);
  pros::delay(250);
  intake.move(127);
  chassis.turnToHeading(-90, 500);
  chassis.waitUntilDone();
  chassis.moveToPose(80, -26, 90, 1750);
  chassis.waitUntilDone();
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  chassis.moveToPose(45, -26, 270, 3000);
  chassis.waitUntilDone();
  intake.move(0);
}

void redNegative() {
  red_side = true;
  doinker.set_value(false);
  const int forwardPower = 600;   // Forward motor power (0-127)
  const int reversePower = -600;  // Reverse motor power (0-127)
  const int reverseTimeMs = 300;  // Reverse duration in ms

  // Start a task to handle motor stall detection
  pros::Task motorTask([&]() {
    handleMotorStall(secondStage, forwardPower, reversePower, reverseTimeMs);
  });
  wall_stake.tare_position();
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(0, 0, 0);

  // Start route
  chassis.moveToPose(-11.5, -27.5, 36.33, 1750,
                     {.forwards = false});  // Move to first mogo
  chassis.waitUntilDone();
  mogo.set_value(true);  // Clamp mogo
  pros::delay(250);
  intake.move(600);
  chassis.moveToPose(9.37, -28.99, 88.44, 1750);
  chassis.waitUntilDone();
  chassis.moveToPose(11.53, -40, 177.66, 3000);
  chassis.waitUntilDone();
  chassis.moveToPose(9.20, -8.45, 177.66, 1500, {.forwards = false});
  chassis.waitUntilDone();
  chassis.turnToHeading(268, 500);
  chassis.waitUntilDone();
  pros::Task top(intakeAutomation);
  chassis.moveToPose(44.15, -11.21, 268.1, 5000);
}

void skills2() {
  red_side = true;
  doinker.set_value(false);
  const int forwardPower = 127;   // Forward motor power (0-127)
  const int reversePower = -127;  // Reverse motor power (0-127)
  const int reverseTimeMs = 300;  // Reverse duration in ms

  // Start a task to handle motor stall detection
  /*pros::Task motorTask([&]() {
    handleMotorStall(secondStage, forwardPower, reversePower, reverseTimeMs);
  });*/
  wall_stake.set_brake_mode(pros::MotorBrake::brake);
  wall_stake.tare_position();
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(0, 0, 0);

  // Start route
  wallThings(175, false);
  chassis.moveToPoint(0, -6.6, 750);
  chassis.waitUntilDone();
  chassis.turnToHeading(-90, 500);
  chassis.waitUntilDone();
  chassis.moveToPoint(10, -6.6, 1500, {.forwards = false});
  chassis.waitUntilDone();
  pros::Task calibrate(calibrateWallStake);
  mogo.set_value(true);
  intake.move(600);
  intake.move(600);
  chassis.moveToPose(15, -27, -204.5, 1250);
  chassis.waitUntilDone();
  chassis.moveToPose(38.5, -83, -190, 5250, {.minSpeed = 40});
  chassis.waitUntil(30);
  wallThings(31, false);
  chassis.waitUntilDone();
  pros::delay(250);
  chassis.moveToPose(35.25, -55.8, -197.8, 1500, {.forwards = false});
  chassis.waitUntilDone();
  chassis.turnToHeading(-267, 500);
  chassis.waitUntilDone();
  intake.move_absolute(-500, 600);
  wallThings(45, false);
  intake.move(60);
  chassis.moveToPose(53.8, -56.5, -266.5, 1250);
  chassis.waitUntilDone();
  pros::Task motorTask([&]() {
    handleMotorStall(secondStage, forwardPower, reversePower, reverseTimeMs);
  });
  intake.move(0);
  wallThings(140, false);
  chassis.moveToPose(40.9, -56.5, -266.5, 1000, {.forwards = false});
  chassis.waitUntilDone();
  pros::Task calibrate2(calibrateWallStake);
  chassis.turnToHeading(-353.25, 750);
  chassis.waitUntilDone();
  intake.move(600);
  intake.move(600);
  chassis.moveToPose(42.9, -36.5, -358.1, 1000);
  chassis.waitUntilDone();
  chassis.moveToPose(44.25, 3.6, -356.35, 2000);
  chassis.waitUntilDone();

  intake.move(0);
  chassis.waitUntilDone();
  pros::delay(1000000);  // Capture focus in auto
}

void skills() {
  red_side = true;
  doinker.set_value(false);
  const int forwardPower = 600;   // Forward motor power (0-127)
  const int reversePower = -600;  // Reverse motor power (0-127)
  const int reverseTimeMs = 400;  // Reverse duration in ms

  // Start a task to handle motor stall detection
  /*pros::Task motorTask([&]() {
    handleMotorStall(secondStage, forwardPower, reversePower, reverseTimeMs);
  });*/
  wall_stake.set_brake_mode(pros::MotorBrake::brake);
  wall_stake.tare_position();
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(0, 0, 0);

  // Start route
  wallThings(170, false);
  chassis.moveToPoint(0, -5.5, 750);
  chassis.waitUntilDone();
  chassis.moveToPose(15.6, -7.3, -91, 1500, {.forwards = false});
  pros::Task calibrate(calibrateWallStake);
  chassis.waitUntilDone();
  mogo.set_value(true);
  intake.move(600);
  chassis.moveToPose(22, -30.45, -225.4, 1500);
  chassis.waitUntilDone();
  chassis.moveToPose(44.9, -82, -196.65, 2000);
  chassis.waitUntil(29);
  wallThings(33, false);
  chassis.waitUntilDone();
  chassis.moveToPose(40.55, -56.2, -188.4, 1500, {.forwards = false});
  chassis.waitUntilDone();
  chassis.turnToHeading(-267, 500);
  chassis.waitUntilDone();
  intake.move_relative(-500, 600);
  chassis.moveToPose(57.85, -57.28, -268.4, 1500);
  wallThings(70, false);
  intake.move(300);
  chassis.waitUntilDone();
  wallThings(150, false);
  chassis.moveToPose(44.43, -56.27, -266., 1000, {.forwards = false});
  chassis.waitUntilDone();
  chassis.turnToHeading(-354, 500);
  pros::Task calibrate2(calibrateWallStake);
  chassis.waitUntilDone();
  intake.move(100);
  chassis.moveToPose(50.4, 1.25, -355, 3000, {.maxSpeed = 75});
  chassis.waitUntilDone();
  chassis.turnToHeading(-224.3, 500);
  chassis.waitUntilDone();
  intake.move(600);
  chassis.moveToPose(58.3, -8.37, -221.7, 1000);
  chassis.waitUntilDone();
  chassis.moveToPose(60.7, 2, -132.7, 1000, {.forwards = false});
  chassis.waitUntilDone();
  mogo.set_value(false);
  chassis.moveToPoint(48.4, -9.28, 1000);
  chassis.waitUntilDone();
  chassis.turnToHeading(-265.5, 1000);
  chassis.waitUntilDone();
  chassis.moveToPose(-16, -5.15, -267.5, 2000, {.forwards = false});
  chassis.waitUntilDone();
  mogo.set_value(true);
  chassis.moveToPose(-22.67, -29.8, -121.6, 1000);
  chassis.waitUntilDone();
  chassis.moveToPose(-45.63, -75.92, -159.83, 2000);
  chassis.waitUntil(29);
  wallThings(33, false);
  chassis.waitUntilDone();
  chassis.moveToPose(-39.25, -50.9, -148.04, 2000, {.forwards = false});
  chassis.waitUntilDone();
  chassis.turnToHeading(-86, 500);
  chassis.waitUntilDone();
  chassis.moveToPose(-59.5, -52.4, -85.82, 1500);
  chassis.waitUntilDone();
  wallThings(150, false);
  chassis.moveToPoint(-47.38, -52.91, 750, {.forwards = false});
  chassis.waitUntilDone();
  chassis.turnToHeading(0, 500);
  chassis.waitUntilDone();
  chassis.moveToPose(-47, 3.0, 1.9, 3000);
  chassis.waitUntilDone();
  chassis.turnToHeading(-122, 500);
  chassis.waitUntilDone();
  chassis.moveToPose(-57.1, -3.63, -124.12, 1000);
  chassis.waitUntilDone();
  chassis.moveToPose(-58.12, 6.6, -227.75, 1000, {.forwards = false});
  chassis.waitUntilDone();
  mogo.set_value(false);

  intake.move(0);
  chassis.waitUntilDone();
  pros::delay(1000000);  // Capture focus in auto
}