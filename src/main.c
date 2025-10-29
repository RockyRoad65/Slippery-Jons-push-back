#include <string.h>
#include "main.h"
#include "drive.h"
#include "autonomous.h"

#define LEFT_FRONT_PORT 15 //reverse
#define LEFT_MIDDLE_PORT 16 // reverse
#define LEFT_REAR_PORT 17 
#define RIGHT_FRONT_PORT 11
#define RIGHT_MIDDLE_PORT 13
#define RIGHT_REAR_PORT 14 // reverse

#define LOWER_INTAKE_PORT 20
#define MIDDLE_INTAKE_PORT 2
#define BACK_INTAKE_PORT 3 

int8_t middle_intake_speed = 127; // we need to speed this up later and also change the gear ratio for the back intake so it's faster but this will keep it from getting stuck rolling on both.

// #define INERTIAL_SENSOR 4
// #define TRACKING_WHEEL_ROTATIONAL_SENSOR 9

#define MATCH_LOADER 'H'
#define HOOD 'A'
#define DESCORE_MECH 'B' // descores the balls from the long goals


void set_intake(bool intake_power, bool main_intake_reversed, bool back_intake_roller_reversed) {
  if (intake_power) {
      if (main_intake_reversed) {
        motor_move(LOWER_INTAKE_PORT, -127);
        motor_move(MIDDLE_INTAKE_PORT, middle_intake_speed);
        motor_move(BACK_INTAKE_PORT, 127);
      } else if (!main_intake_reversed) {
        motor_move(LOWER_INTAKE_PORT, 127);
        motor_move(MIDDLE_INTAKE_PORT, -1*middle_intake_speed);

        // control the back intake roller separately from the rest of the intake so that the intake can be going forward, moving the balls up and through while at the same time the back intake roller can spin reversed to score on the center goal if desired since they're lower than the long goals.
        if (!back_intake_roller_reversed) motor_move(BACK_INTAKE_PORT, -127);
        else if (back_intake_roller_reversed) motor_move(BACK_INTAKE_PORT, 127);
      }

    } else if(!intake_power) {
      motor_move(LOWER_INTAKE_PORT, 0);
      motor_move(MIDDLE_INTAKE_PORT, 0);
      motor_move(BACK_INTAKE_PORT, 0);
    }
}


void on_center_button() {}

void initialize() {
  adi_port_set_config(MATCH_LOADER, E_ADI_DIGITAL_OUT);
  adi_port_set_config(HOOD, E_ADI_DIGITAL_OUT);
  adi_port_set_config(DESCORE_MECH, E_ADI_DIGITAL_OUT);
  if (usd_is_installed() == 0) {
    controller_print(E_CONTROLLER_MASTER, 0, 0, "No uSD card!");
  } else if (!competition_is_connected() && usd_is_installed() == 1) {
    controller_print(E_CONTROLLER_MASTER, 0, 0, "B to record auton..");
  }
  adi_digital_write(MATCH_LOADER, false);
  imu_reset_blocking(4);
  rotation_reset(9);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  controller_clear(E_CONTROLLER_MASTER);
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


void autonomous() {
// set later
}


void opcontrol() {
  bool match_loader_extended = false, hood_extended = false, descore_mech_extended = false;
  uint32_t count = 0;
  controller_clear(E_CONTROLLER_MASTER); // Clear all lines of the controller screen so the whole screen can be displayed to
  while (true) {
    
    move_drivetrain(LEFT_FRONT_PORT, LEFT_REAR_PORT, LEFT_MIDDLE_PORT, RIGHT_FRONT_PORT, RIGHT_REAR_PORT, RIGHT_MIDDLE_PORT, (controller_get_analog(CONTROLLER_MASTER, ANALOG_LEFT_Y))/127.0f, controller_get_analog(CONTROLLER_MASTER, ANALOG_RIGHT_X)/127.0f);

    // update later to what Jonathan wants
    if (controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L1)) {
      set_intake(true, false, false); // intake all spins forward
    } else if (controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L2)) {
      set_intake(true, false, true); // intake spins forward except the back roller spins in reverse
    } else if (controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_R1)) {
      set_intake(true, true, true); // intake all spins backward
    } else {
      set_intake(false, false, false); // intake off 
    }
    
    // Toggle the match unloader piston when X is pressed on the controller
    if (controller_get_digital_new_press(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_X)) { 
      match_loader_extended = !match_loader_extended;
      adi_digital_write(MATCH_LOADER, match_loader_extended);
    } // Toggle the hood pneumatics when A is pressed on the controller
    if (controller_get_digital_new_press(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_A)) { 
      hood_extended = !hood_extended;
      adi_digital_write(HOOD, hood_extended);
    }
    if (controller_get_digital_new_press(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_Y)) {
      descore_mech_extended = !descore_mech_extended;
      adi_digital_write(DESCORE_MECH, descore_mech_extended);
    }

    // Update Controller Screen
    if (count % 400 == 200 && !competition_is_connected() && usd_is_installed() == 1) {
      controller_print(E_CONTROLLER_MASTER, 0, 0, "B to record auton..");
    } else if (count % 400 == 0 && usd_is_installed() == 1) {
      controller_print(E_CONTROLLER_MASTER, 0, 0, "up--auton selector");
    } else if (count % 10 == 0) {
        controller_print(E_CONTROLLER_MASTER, 1, 0, "brain: %f", battery_get_capacity());
    } else if (count % 10 == 5) {
      if (hood_extended) { controller_print(E_CONTROLLER_MASTER, 2, 0, "hood shut");
      } else if (!hood_extended) { controller_print(E_CONTROLLER_MASTER, 2, 0, "hood open"); }
    }

    count++;

    delay(10);
  }
}
