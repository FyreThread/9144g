#include "main.h"

#include <cstdio>

#include "./lemlib/api.hpp"  // IWYU pragma: keep
#include "autons.hpp"
#include "devices.h"                             // Include the header file where the intake_sensor is declared
#include "lemlib/chassis/chassis.hpp"            // IWYU pragma:  keep
#include "lemlib/chassis/trackingWheel.hpp"      // IWYU pragma:  keep
#include "liblvgl/extra/layouts/flex/lv_flex.h"  // IWYU pragma:  keep
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"  // IWYU pragma:  keep
#include "pros/adi.hpp"             // IWYU pragma:  keep
#include "pros/device.hpp"          // IWYU pragma:  keep
#include "pros/misc.h"              // IWYU pragma:  keep
#include "pros/motor_group.hpp"     // IWYU pragma:  keep
#include "pros/motors.h"            // IWYU pragma:  keep
#include "pros/motors.hpp"          // IWYU pragma:  keep
#include "pros/rtos.hpp"            // IWYU pragma:  keep
#include "subroutines.h"            // IWYU pragma:  keep