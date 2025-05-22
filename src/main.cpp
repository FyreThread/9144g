#include "autons.hpp"
#include "devices.h"
#include "includes.h"
#include "pros/abstract_motor.hpp"  // IWYU pragma: keep
#include "pros/misc.h"
#include "subroutines.h"  // IWYU pragma: keep

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

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"Literally Nothing\n", skills},
      {"Red Solo Win Point\n", redSWP},
      {"Blue Solo Win Point\n", blueSWP},
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
      if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        autonomous();
      }
    }

    // Controls

    // Intake Controls
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intake.move(127);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      intake.move(-127);
    } else {
      intake.move(0);
    }
    // Top Roller Controls
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      top_roller.move(-127);
    }

    pros::delay(25);  // Delay for the poor IC
  }
}