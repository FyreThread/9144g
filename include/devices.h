#pragma once
#include <cstdio>

#include "./lemlib/api.hpp"  // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/extra/layouts/flex/lv_flex.h"  // IWYU pragma:  keep
#include "pros/adi.hpp"                          // IWYU pragma: keep
#include "pros/device.hpp"                       // IWYU pragma:  keep
#include "pros/misc.h"                           // IWYU pragma:  keep
#include "pros/motor_group.hpp"
#include "pros/motors.h"     // IWYU pragma:  keep
#include "pros/motors.hpp"   // IWYU pragma: keep
#include "pros/optical.hpp"  // IWYU pragma:  keep
#include "pros/rotation.hpp"

namespace my_robot {

extern pros::MotorGroup intake;

extern pros::Motor top_roller;

// left motors
extern pros::MotorGroup left_motors;
// right motors
extern pros::MotorGroup right_motors;
// drivetrain
extern lemlib::Drivetrain drivetrain;

// inertial
extern pros::Imu imu;
// create a vertical on port 1
extern pros::Rotation vertical_encoder;
// create a vertical on port 1
extern pros::Rotation horizontal_encoder;
// vertical tracking wheel
extern lemlib::TrackingWheel vertical_tracking_wheel;
// vertical tracking wheel
extern lemlib::TrackingWheel horizontal_tracking_wheel;
// setup odom
extern lemlib::OdomSensors sensors;

// PIDs
// lateral PID controller
extern lemlib::ControllerSettings lateral_controller;

// angular PID controller
extern lemlib::ControllerSettings angular_controller;
// create the chassis
extern lemlib::Chassis chassis;

}  // namespace my_robot