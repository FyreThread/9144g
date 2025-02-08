#include "subroutines.h"

#include "devices.h"  // IWYU pragma:  keep
#include "main.h"     // IWYU pragma:  keep

namespace my_robot {

void handleMotorStall(pros::Motor &motor, int forwardPower, int reversePower, int reverseTimeMs) {
  const int stallThreshold = 2000;    // Adjust based on your motor's typical stalled current (in mA)
  const int stallCheckInterval = 50;  // Interval to check motor status (ms)
  const int stallDurationMs = 500;    // Duration the motor must be stalled before reacting (ms)

  int stallCounter = 0;

  while (true) {
    // Check motor current draw
    int currentDraw = motor.get_current_draw();

    if (currentDraw > stallThreshold) {
      // Increment stall counter if current exceeds threshold
      stallCounter += stallCheckInterval;

      // If the motor has been stalled for the set duration
      if (stallCounter >= stallDurationMs) {
        // Reverse the motor briefly
        motor.move(reversePower);
        pros::delay(reverseTimeMs);

        // Resume forward motion
        motor.move(forwardPower);
        stallCounter = 0;  // Reset stall counter
      }
    } else {
      // Reset stall counter if motor is not stalled
      stallCounter = 0;
    }

    pros::delay(stallCheckInterval);  // Wait before the next check
  }
}

void stopIntakeOnSecondPress() {
  static int pressCount = 0;       // Counter to track the number of presses
  static bool wasPressed = false;  // Tracks whether the switch was previously pressed

  while (true) {
    // Check if the limit switch is pressed
    bool isPressed = intake_sensor.get_value();

    // If the switch transitions from not pressed to pressed
    if (isPressed && !wasPressed) {
      pressCount++;

      // Stop the intake motor on the second press
      if (pressCount == 2) {
        intake.move(0);
        pressCount = 0;  // Reset press count
      }
    }

    wasPressed = isPressed;  // Update the previous state
    pros::delay(20);         // Delay to prevent rapid toggling
  }
}

void stopIntakeOnFirstPress() {
  static bool wasPressed = false;  // Tracks whether the switch was previously pressed

  while (true) {
    // Check if the limit switch is pressed
    bool isPressed = intake_sensor.get_value();

    // If the switch transitions from not pressed to pressed
    if (isPressed && !wasPressed) {
      intake.move(0);  // Stop the intake motor
    }

    wasPressed = isPressed;  // Update the previous state
    pros::delay(20);         // Delay to prevent rapid toggling
  }
}

#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

// Adjustable values
const int SECOND_STAGE_ROTATION = 450;  // Degrees to spin secondStage
const int DELAY_AFTER_STAGE = 300;      // Delay before resuming intake (ms)
const int INTAKE_SPEED = 600;           // Intake motor speed
const int POSITION_TOLERANCE = 5;       // Allowed error in degrees

void intakeAutomation() {
  bool wasPressed = false;
  bool hasRun = false;  // Flag to track if the automation has already run

  while (true) {
    bool isPressed = intake_sensor.get_value();  // Read sensor state

    pros::lcd::print(1, "Sensor Value: %d", isPressed);

    if (isPressed && !wasPressed &&
        !hasRun) {  // Only trigger on the first press
      pros::lcd::set_text(2, "Sensor Triggered!");

      intake.move(0);                                         // Stop intake temporarily
      secondStage.move_relative(SECOND_STAGE_ROTATION, 600);  // Move secondStage

      // Wait until secondStage stops moving
      while (std::abs(secondStage.get_actual_velocity()) > 1) {
        pros::lcd::print(3, "Motor Speed: %.2f",
                         secondStage.get_actual_velocity());
        pros::delay(10);
      }

      pros::lcd::set_text(4, "SecondStage Done!");
      pros::delay(DELAY_AFTER_STAGE);

      // Resume intake at full speed
      intake.move_velocity(INTAKE_SPEED);

      hasRun = true;  // Mark that the automation has run once
    }

    wasPressed = isPressed;
    pros::delay(20);
  }
}

// Add your function definitions here

void wallThings(int number) {
  wall_stake.move_absolute(number, 127);
  while (!((wall_stake.get_position() < (number + 7)) &&
           (wall_stake.get_position() > (number - 7)))) {
    // Continue running this loop as long as the motor is not within +-5 units
    // of its goal
    pros::delay(2);
  }
  calibrateWallStake();
}

void calibrateWallStake() {
  const int timeoutMs = 3000;    // Max time to attempt calibration (3 seconds)
  const int moveSpeed = -50;     // Speed for moving the stake down
  const int checkInterval = 10;  // Time between sensor checks (ms)

  int elapsedTime = 0;  // Track elapsed time

  // Move the wall stake down
  wall_stake.move(moveSpeed);

  // Wait until the wall sensor is triggered or timeout is reached
  while (!wall_sensor.get_value() && elapsedTime < timeoutMs) {
    pros::delay(checkInterval);
    elapsedTime += checkInterval;
  }

  // Stop the motor
  wall_stake.move(0);

  // If the sensor was triggered, set the current position as zero
  if (wall_sensor.get_value()) {
    wall_stake.tare_position();
    pros::lcd::print(3, "Wall stake calibrated!");
  } else {
    pros::lcd::print(3, "Calibration timeout! Check wall stake.");
  }
}

// Function to detect and react to rings
void colorSort(int waitTimeMs, int stopTimeMs, bool red_side) {
  color_sort.set_led_pwm(100);  // Ensure the sensor LED is on

  while (true) {
    pros::c::optical_rgb_s_t color_data = color_sort.get_rgb();
    int detected_hue = color_sort.get_hue();

    bool blue_detected = (detected_hue > 180 && detected_hue < 260);
    bool red_detected = (detected_hue > 0 && detected_hue < 60);

    if ((red_side && blue_detected) || (!red_side && red_detected)) {
      // Stop the intake for the designated time
      pros::delay(waitTimeMs);
      secondStage.move_velocity(0);
      pros::delay(stopTimeMs);
      secondStage.move_velocity(600);  // Resume intake
    }

    pros::delay(20);  // Reduce CPU usage
  }
}

}  // namespace my_robot