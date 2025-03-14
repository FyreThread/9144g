#include "autons.hpp"
#include "includes.h"
#include "subroutines.h"

using namespace my_robot;

void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

void initialize() {
  pros::lcd::initialize();                          // Initialize brain screen
  chassis.calibrate();                              // Calibrate sensors
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);  // Set brake mode for chassis motors
  wall_stake.set_brake_mode(MOTOR_BRAKE_HOLD);      // Set brake mode for wall stake motor

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"AAAAAAAAAAh\n", PIDtune},
      {"Red Solo Win Point\n", redSWP},
      {"Blue Solo Win Point\n", blueSWP},
      {"Red Negative\n", redNegative},
      {"Skills\n", skills},
  });

  ez::as::initialize();  // Initialize autonomous selector

  // Print position to brain screen
  pros::Task screen_task([&]() {
    while (true) {
      // Print robot location to the brain screen
      pros::lcd::print(5, "X: %f", chassis.getPose().x);          // X-coordinate
      pros::lcd::print(6, "Y: %f", chassis.getPose().y);          // Y-coordinate
      pros::lcd::print(7, "Theta: %f", chassis.getPose().theta);  // Heading
      pros::delay(20);                                            // Delay to save resources
    }
  });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  ez::as::auton_selector.selected_auton_call();  // Calls selected autonomous routine from autonomous selector
}

// OPERATOR CONTROL
pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
  chassis.cancelAllMotions();  // Cancel all ongoing motions
  while (true) {
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);    // Get left joystick Y-axis value
    int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);    // Get left joystick X-axis value
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);  // Get right joystick X-axis value

    chassis.arcade(leftY, (leftX + rightX) * .7);  // Control chassis with arcade drive

    // Check if the robot is not connected to the competition control
    if (!pros::competition::is_connected()) {
      // If both DIGITAL_UP and DIGITAL_LEFT buttons are pressed, run autonomous routine
      if (controller.get_digital(DIGITAL_A)) {
        autonomous();
      }
    }

    // Control intake motor
    if (controller.get_digital(DIGITAL_R2))
      intake.move(127);  // Move intake forward
    else if (controller.get_digital(DIGITAL_L2))
      intake.move(-127);  // Move intake backward
    else
      intake.move(0);  // Stop intake

    // Control doinker mechanism
    if (controller.get_digital(DIGITAL_RIGHT))
      doinker.set_value(true);  // Activate doinker
    if (controller.get_digital(DIGITAL_DOWN))
      doinker.set_value(false);  // Deactivate doinker

    // Control mobile goal mechanism
    if (controller.get_digital(DIGITAL_R1))
      mogo.set_value(true);  // Activate mogo
    else if (controller.get_digital(DIGITAL_L1))
      mogo.set_value(false);  // Deactivate mogo

    // Define a tolerance for holding position
    const double POSITION_TOLERANCE = 2.0;

    // Store the last commanded position
    static double target_position = wall_stake.get_position();

    if (controller.get_digital_new_press(DIGITAL_Y)) {
      target_position = 150;  // Set new target position
      wall_stake.move_absolute(target_position, 110);

      // Wait until the motor reaches the position, but allow UP/LEFT override
      while (std::abs(wall_stake.get_position() - target_position) > POSITION_TOLERANCE &&
             !controller.get_digital(DIGITAL_UP) && !controller.get_digital(DIGITAL_LEFT)) {
        pros::delay(10);  // Small delay to prevent CPU overload
      }
    } else if (controller.get_digital_new_press(DIGITAL_B)) {
      target_position = 34;  // Set new target position
      wall_stake.move_absolute(target_position, 110);

      // Wait until the motor reaches the position, but allow UP/LEFT override
      while (std::abs(wall_stake.get_position() - target_position) > POSITION_TOLERANCE &&
             !controller.get_digital(DIGITAL_UP) && !controller.get_digital(DIGITAL_LEFT)) {
        pros::delay(10);
      }
    } else if (controller.get_digital_new_press(DIGITAL_X)) {
      calibrateWallStake();
    }
    // Manual movement while holding buttons
    else if (controller.get_digital(DIGITAL_LEFT)) {
      wall_stake.move(-600);  // Move wall stake reverse (only while held)
    } else if (controller.get_digital(DIGITAL_UP)) {
      wall_stake.move(600);  // Move wall stake forward (only while held)
    }
    // Stop motor if UP or LEFT was released, but hold position when reaching target
    else {
      wall_stake.move_velocity(0);  // Hold position when reaching target
    }

    pros::delay(25);  // Delay for the poor IC
  }
}