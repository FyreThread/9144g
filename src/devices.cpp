#include "devices.h"  // Include the header file where the intake_sensor is declared

#include <cstdio>

#include "./lemlib/api.hpp"  // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/extra/layouts/flex/lv_flex.h"  // IWYU pragma:  keep
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/device.hpp"  // IWYU pragma:  keep
#include "pros/misc.h"      // IWYU pragma:  keep
#include "pros/motor_group.hpp"
#include "pros/motors.h"  // IWYU pragma:  keep
#include "pros/motors.hpp"
#include "pros/optical.hpp"  // IWYU pragma:  keep

namespace my_robot {

// Define motor groups and individual motors
pros::MotorGroup intake({6, -12});  // Intake motor group on ports 6 and -12
pros::Motor firstStage(-12,
                       pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);  // First stage motor
pros::v5::Motor secondStage(6,
                            pros::v5::MotorGears::blue);  // Second stage motor
pros::adi::DigitalOut mogo('C', false);                   // Mobile goal mechanism on port 'C'
pros::adi::DigitalOut doinker('D', false);                // Doinker mechanism on port 'D'
pros::adi::DigitalIn intake_sensor('H');                  // Intake sensor on port 'H'
pros::adi::DigitalIn wall_sensor('G');                    // Wall sensor on port 'G'
pros::Optical color_sort(15);

// Define wall stake motor
pros::Motor wall_stake(-1, pros::v5::MotorGears::red, pros::v5::MotorUnits::degrees);  // Wall stake motor on port -1

// Define left and right motor groups for the drivetrain
pros::MotorGroup left_motors({-2, 3, -4}, pros::MotorGearset::blue);   // Left motors on ports -2, 3, -4
pros::MotorGroup right_motors({7, -9, 10}, pros::MotorGearset::blue);  // Right motors on ports 7, -9, 10

// Define the drivetrain
lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 12.0, lemlib::Omniwheel::NEW_325, 400, 8);

// Define the inertial sensor
pros::Imu imu(5);  // Inertial sensor on port 5

// Define the vertical encoder
pros::adi::Encoder vertical_encoder('A', 'B', true);  // Optical shaft encoder on ports 'A' and 'B'

// Define the vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, .45);

// Setup odometry sensors
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel,  // Vertical tracking wheel 1
    nullptr,                   // Vertical tracking wheel 2 (not used)
    nullptr,                   // Horizontal tracking wheel 1 (not used)
    nullptr,                   // Horizontal tracking wheel 2 (not used)
    &imu                       // Inertial sensor
);

// Define PID controllers
lemlib::ControllerSettings lateral_controller(
    6.9,  // Proportional gain (kP)
    .05,  // Integral gain (kI)
    5.5,  // Derivative gain (kD)
    3,    // Anti windup
    .25,  // Small error range, in inches
    25,   // Small error range timeout, in milliseconds
    0,    // Large error range, in inches
    0,    // Large error range timeout, in milliseconds
    17    // Maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(
    2.245,  // Proportional gain (kP)
    0.00,   // Integral gain (kI)
    15,     // Derivative gain (kD)
    0,      // Anti windup
    0,      // Small error range, in degrees
    0,      // Small error range timeout, in milliseconds
    0,      // Large error range, in degrees
    0,      // Large error range timeout, in milliseconds
    0       // Maximum acceleration (slew)
);

// Create the chassis
lemlib::Chassis chassis(
    drivetrain,          // Drivetrain settings
    lateral_controller,  // Lateral PID settings
    angular_controller,  // Angular PID settings
    sensors              // Odometry sensors
);

}  // namespace my_robot
