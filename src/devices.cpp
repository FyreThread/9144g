#include "devices.h"  // Include the header file where the intake_sensor is declared

#include <cstdio>

#include "./lemlib/api.hpp"  // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/extra/layouts/flex/lv_flex.h"  // IWYU pragma:  keep
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"     // IWYU pragma: keep
#include "pros/device.hpp"  // IWYU pragma:  keep
#include "pros/misc.h"      // IWYU pragma:  keep
#include "pros/motor_group.hpp"
#include "pros/motors.h"     // IWYU pragma:  keep
#include "pros/motors.hpp"   // IWYU pragma: keep
#include "pros/optical.hpp"  // IWYU pragma:  keep
#include "pros/rotation.hpp"

namespace my_robot {

// Define motor groups and individual motors
pros::MotorGroup intake({-1, 10, -17, -9});  // Intake motor group on ports 6 and -12

pros::Motor top_roller(-9);

// Define left and right motor groups for the drivetrain
pros::MotorGroup left_motors({-20, -19, 18}, pros::MotorGearset::blue);   // Left motors on ports -2, 3, -4
pros::MotorGroup right_motors({13, 12, -14}, pros::MotorGearset::blue);  // Right motors on ports 7, -9, 10

// Define the drivetrain
lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 12.0, lemlib::Omniwheel::NEW_325, 450, 8);

// Define the inertial sensor
pros::Imu imu(15);  // Inertial sensor on port 5

// Define the vertical encoder
pros::Rotation vertical_encoder(5);   // Optical shaft encoder on ports 'A' and 'B'
pros::Rotation horizontal_encoder(6);  // Optical shaft encoder on ports 'A' and 'B'

// Define the vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, 0.5);
// Define the vertical tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, .125);

// Setup odometry sensors
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel,    // Vertical tracking wheel 1
    nullptr,                     // Vertical tracking wheel 2 (not used)
    &horizontal_tracking_wheel,  // Horizontal tracking wheel 1 (not used)
    nullptr,                     // Horizontal tracking wheel 2 (not used)
    &imu                         // Inertial sensor
);

// Define PID controllers
lemlib::ControllerSettings lateral_controller(
    6.9,  // Proportional gain (kP)
    0,    // Integral gain (kI)
    5.5,  // Derivative gain (kD)
    0,    // Anti windup
    0,    // Small error range, in inches
    0,    // Small error range timeout, in milliseconds
    0,    // Large error range, in inches
    0,    // Large error range timeout, in milliseconds
    17    // Maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(
    2.95,  // Proportional gain (kP)
    0.00,  // Integral gain (kI)
    23,    // Derivative gain (kD)
    0,     // Anti windup
    0,     // Small error range, in degrees
    0,     // Small error range timeout, in milliseconds
    0,     // Large error range, in degrees
    0,     // Large error range timeout, in milliseconds
    0      // Maximum acceleration (slew)
);

// Create the chassis
lemlib::Chassis chassis(
    drivetrain,          // Drivetrain settings
    lateral_controller,  // Lateral PID settings
    angular_controller,  // Angular PID settings
    sensors              // Odometry sensors
);

}  // namespace my_robot
