#include "devices.h"
#include "main.h"  // IWYU pragma: keep
#include "pros/motors.h"
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
  const int waitTime = 500;
  const int stopTime = 125;

  // Start a task to handle motor stall detection
  pros::Task motorTask([&]() {
    handleMotorStall(secondStage, forwardPower, reversePower, reverseTimeMs);
  });
  pros::Task cs([&]() {
    colorSort(waitTime, stopTime, red_side);
  });
  wall_stake.tare_position();
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(0, 0, 0);

  // Start route
  wallThings(170, false);
  chassis.moveToPose(-20.8, -28.78, 90.67, 1600,
                     {.forwards = false});  // Move to first mogo
  chassis.waitUntilDone();
  mogo.set_value(true);  // Clamp mogo
  intake.move(127);
  pros::Task calibrate(calibrateWallStake);
  chassis.moveToPose(-29.32, -51.8, 200, 2500);
  chassis.waitUntilDone();
  pros::delay(250);
  chassis.moveToPose(-31.33, -38.74, 117.85, 1500, {.forwards = false});
  chassis.waitUntilDone();
  chassis.moveToPose(-16.31, -45.65, 111, 1000);
  chassis.waitUntilDone();
  chassis.moveToPose(-14.1, -8.65, -20.3, 2000);
  chassis.waitUntilDone();
  mogo.set_value(false);
  // It needs to stop on the second red here
  chassis.moveToPose(-20.4, 16.86, -8.2, 2000);
  chassis.waitUntilDone();
  chassis.moveToPose(-46.02, 12.95, 90, 2000, {.forwards = false});
  chassis.waitUntilDone();
  mogo.set_value(true);
  chassis.moveToPose(-62.75, 30.59, -31.6, 2000);
  chassis.waitUntilDone();
  chassis.moveToPose(-52.12, 9.55, -161.65, 2000);
  chassis.waitUntilDone();
  wall_stake.move_absolute(160, 127);
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

void skills() {
  red_side = true;
  doinker.set_value(false);
  const int forwardPower = 600;   // Forward motor power (0-127)
  const int reversePower = -600;  // Reverse motor power (0-127)
  const int reverseTimeMs = 400;  // Reverse duration in ms
  const int waitTime = 500;
  const int stopTime = 125;

  // Start a task to handle motor stall detection
  /*pros::Task motorTask([&]() {
    handleMotorStall(secondStage, forwardPower, reversePower, reverseTimeMs);
  });*/
  wall_stake.set_brake_mode(pros::MotorBrake::brake);
  wall_stake.tare_position();
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(0, -1.375, 0);

  // Start route
  wall_stake.move_absolute(170, 100);
  chassis.moveToPoint(0, -5.5, 750, {.maxSpeed = 30});
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
  wallThings(35, false);
  chassis.waitUntilDone();
  chassis.moveToPose(40.55, -56.2, -188.4, 1500, {.forwards = false});
  chassis.waitUntilDone();
  chassis.turnToHeading(-267, 500);
  chassis.waitUntilDone();
  intake.move_relative(-100, 600);
  intake.move_relative(200, 600);
  intake.move_relative(-500, 600);
  chassis.moveToPose(56.85, -57.28, -268.4, 2500);
  wallThings(45, false);
  chassis.waitUntilDone();
  wallThings(150, false);
  wallThings(150, false);

  /*chassis.moveToPoint(-47.38, -52.91, 750, {.forwards = false});


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


  calibrateWallStake();


  pros::Task cs([&]() { colorSort(waitTime, stopTime, red_side); });


  chassis.moveToPose(-50, -97.15, -180.36, 4000);


  wallThings(33, false);


  chassis.waitUntilDone();


  chassis.turnToHeading(-88.77, 500);


  chassis.waitUntilDone();


  intake.move_relative(-500, 100);


  wallThings(70, false);


  chassis.moveToPose(-4.88, -100, -90.65, 2000, {.forwards = false});


  chassis.waitUntilDone();


  mogo.set_value(true);


  chassis.turnToHeading(-179, 500);


  chassis.waitUntilDone();


  chassis.moveToPoint(-6.5, -115.5, 2000, {.maxSpeed = 40});


  chassis.waitUntilDone();


  chassis.moveToPose(-6.5, -107.82, -179, 1000, {.forwards = false});


  chassis.waitUntilDone();
  wallThings(203, false);*/

  intake.move(0);
  chassis.waitUntilDone();
  pros::delay(1000000);  // Capture focus in auto
}