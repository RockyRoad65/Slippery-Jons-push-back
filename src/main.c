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

// #define INERTIAL_SENSOR 4
// #define TRACKING_WHEEL_ROTATIONAL_SENSOR 9

#define MOGO 'H'
#define DOINKER 'A'

void toggle_intake(bool intake_power, bool intake_reversed) {
  if (intake_power) {
      if (intake_reversed) {
        motor_move(LOWER_INTAKE_PORT, -127);
        motor_move(MIDDLE_INTAKE_PORT, 127);
        motor_move(BACK_INTAKE_PORT, 127);
      } else if (!intake_reversed) {
        motor_move(LOWER_INTAKE_PORT, 127);
        motor_move(MIDDLE_INTAKE_PORT, -127);
        motor_move(BACK_INTAKE_PORT, -127);
      }
    } else if(!intake_power) {
      motor_move(LOWER_INTAKE_PORT, 0);
      motor_move(MIDDLE_INTAKE_PORT, 0);
      motor_move(BACK_INTAKE_PORT, 0);
    }
}


void on_center_button() {}

void initialize() {
  adi_port_set_config(MOGO, E_ADI_DIGITAL_OUT);
  adi_port_set_config(DOINKER, E_ADI_DIGITAL_OUT);
  if (usd_is_installed() == 0) {
    controller_print(E_CONTROLLER_MASTER, 0, 0, "No uSD card!");
  } else if (!competition_is_connected() && usd_is_installed() == 1) {
    controller_print(E_CONTROLLER_MASTER, 0, 0, "B to record auton..");
  }
  adi_digital_write(MOGO, true);
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
  /* ---------------------------------------- */
  // left side either color scores preload only
  /* ---------------------------------------- */
  rotation_reset(9);
  adi_digital_write(MOGO, true);
  // turn(LEFT_FRONT_PORT, LEFT_REAR_PORT, LEFT_MIDDLE_PORT, RIGHT_FRONT_PORT, RIGHT_REAR_PORT, RIGHT_MIDDLE_PORT, 4, 160);
  toggle_intake(true, true);
  delay(250); // release the front intake part
  toggle_intake(false, false);

  // Score the Preload
  move(LEFT_FRONT_PORT, LEFT_REAR_PORT, LEFT_MIDDLE_PORT, RIGHT_FRONT_PORT, RIGHT_REAR_PORT, RIGHT_MIDDLE_PORT, 1270, true, false);
  delay(50);
  turn(LEFT_FRONT_PORT, LEFT_REAR_PORT, LEFT_MIDDLE_PORT, RIGHT_FRONT_PORT, RIGHT_REAR_PORT, RIGHT_MIDDLE_PORT, 30);
  delay(50);
  move(LEFT_FRONT_PORT, LEFT_REAR_PORT, LEFT_MIDDLE_PORT, RIGHT_FRONT_PORT, RIGHT_REAR_PORT, RIGHT_MIDDLE_PORT, 1700, true, true);
  adi_digital_write(MOGO, false); // turn on the mogo
  delay(150);
  move(LEFT_FRONT_PORT, LEFT_REAR_PORT, LEFT_MIDDLE_PORT, RIGHT_FRONT_PORT, RIGHT_REAR_PORT, RIGHT_MIDDLE_PORT, 1300, false, true);
  toggle_intake(true, false); // make sure it's not too high and will get stuck by briefly reversing the intake
  delay(1000); // delay until the ring is at least on the top of the mogo if not totally scored
  turn(LEFT_FRONT_PORT, LEFT_REAR_PORT, LEFT_MIDDLE_PORT, RIGHT_FRONT_PORT, RIGHT_REAR_PORT, RIGHT_MIDDLE_PORT, 110);
  delay(150);
  move(LEFT_FRONT_PORT, LEFT_REAR_PORT, LEFT_MIDDLE_PORT, RIGHT_FRONT_PORT, RIGHT_REAR_PORT, RIGHT_MIDDLE_PORT, 500, false, true);
  delay(1500);
  toggle_intake(false, false);
  turn(LEFT_FRONT_PORT, LEFT_REAR_PORT, LEFT_MIDDLE_PORT, RIGHT_FRONT_PORT, RIGHT_REAR_PORT, RIGHT_MIDDLE_PORT, 265);
  move(LEFT_FRONT_PORT, LEFT_REAR_PORT, LEFT_MIDDLE_PORT, RIGHT_FRONT_PORT, RIGHT_REAR_PORT, RIGHT_MIDDLE_PORT, 1400, false, true); // back up into the ladder
  motor_move(LEFT_FRONT_PORT, 0);
  motor_move(LEFT_REAR_PORT, 0);
  motor_move(LEFT_MIDDLE_PORT, 0);
  motor_move(RIGHT_FRONT_PORT, 0);
  motor_move(RIGHT_REAR_PORT, 0);
  motor_move(RIGHT_MIDDLE_PORT, 0);

}


void opcontrol() {
  bool mogo_extended = true, doinker_extended = false;
  bool intake_power = false;
  bool intake_reversed = false;
  uint32_t count = 0;
  controller_clear(E_CONTROLLER_MASTER); // Clear all lines of the controller screen so the whole screen can be displayed to
  while (true) {
    
    move_drivetrain(LEFT_FRONT_PORT, LEFT_REAR_PORT, LEFT_MIDDLE_PORT, RIGHT_FRONT_PORT, RIGHT_REAR_PORT, RIGHT_MIDDLE_PORT, (controller_get_analog(CONTROLLER_MASTER, ANALOG_LEFT_Y))/127.0f, controller_get_analog(CONTROLLER_MASTER, ANALOG_RIGHT_X)/127.0f);

    // Toggle the intake or intake directions when bumpers are pressed (UPDATE WHEN THE PHYSICAL DESIGN IS SET)
    if (controller_get_digital_new_press(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L1)) {
      intake_power = !intake_power;
      toggle_intake(intake_power, intake_reversed);
    } else if (controller_get_digital_new_press(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_R1)) {
      intake_reversed = !intake_reversed;
      toggle_intake(intake_power, intake_reversed);
    }
    
    // Toggle the mogo pistons pneumatics when X is pressed on the controller
    if (controller_get_digital_new_press(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_X)) { 
      mogo_extended = !mogo_extended;
      adi_digital_write(MOGO, mogo_extended);
    } // Toggle the doinker pneumatics when A is pressed on the controller
    if (controller_get_digital_new_press(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_A)) { 
      doinker_extended = !doinker_extended;
      adi_digital_write(DOINKER, doinker_extended);
    }

    // Update Controller Screen
    if (count % 400 == 200 && !competition_is_connected() && usd_is_installed() == 1) {
      controller_print(E_CONTROLLER_MASTER, 0, 0, "B to record auton..");
    } else if (count % 400 == 0 && usd_is_installed() == 1) {
      controller_print(E_CONTROLLER_MASTER, 0, 0, "up--auton selector");
    } else if (count % 10 == 0) {
        controller_print(E_CONTROLLER_MASTER, 1, 0, "brain: %f", battery_get_capacity());
    } else if (count % 10 == 5) {
      if (mogo_extended) { controller_print(E_CONTROLLER_MASTER, 2, 0, "piston open ");
      } else if (!mogo_extended) { controller_print(E_CONTROLLER_MASTER, 2, 0, "piston closed "); }
    }

    count++;

    delay(10);
  }
}
