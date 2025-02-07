#include "includes.h"

using namespace my_robot;

void initialize() {
  pros::lcd::initialize();                          // Initialize brain screen
  chassis.calibrate();                              // Calibrate sensors
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);  // Set brake mode for chassis motors
  wall_stake.set_brake_mode(MOTOR_BRAKE_HOLD);      // Set brake mode for wall stake motor

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
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
      pros::lcd::print(0, "X: %f", chassis.getPose().x);          // X-coordinate
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);          // Y-coordinate
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);  // Heading
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
      if (controller.get_digital(DIGITAL_UP) && controller.get_digital(DIGITAL_LEFT)) {
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

    // Control wall stake motor
    if (controller.get_digital(DIGITAL_Y))
      wall_stake.move_absolute(320, 110);  // Move wall stake to score position
    else if (controller.get_digital(DIGITAL_X))
      calibrateWallStake();  // Calibrate wall stake
    else if (controller.get_digital(DIGITAL_B))
      wall_stake.move_absolute(73, 110);  // Move wall stake to load position

    pros::delay(25);  // Delay for the poor IC
  }
}