#include <stdio.h>
#include <math.h>
#include "main.h"
#include "drive.h"

#define LEFT_FRONT_PORT -15 //reverse
#define LEFT_MIDDLE_PORT -16 // reverse
#define LEFT_REAR_PORT 17 
#define RIGHT_FRONT_PORT 11
#define RIGHT_MIDDLE_PORT 13
#define RIGHT_REAR_PORT -14 // reverse

#define LOWER_INTAKE_PORT 20
#define MIDDLE_INTAKE_PORT 2
#define BACK_INTAKE_PORT -3 


#define INERTIAL_SENSOR 4
#define TRACKING_WHEEL_ROTATIONAL_SENSOR 9

#define MATCH_LOADER 'H'
#define HOOD 'A'
#define DESCORE_MECH 'B' // descores the balls from the long goals

using namespace pros::c;
using namespace pros;
// Just so I didn't have to change my original c code as much. It probably should be done eventually

void set_intake(bool intake_power, bool main_intake_reversed, bool back_intake_roller_reversed, int8_t intake_speed) { // intake speed default 127
  if (intake_power) {
      if (main_intake_reversed) {
        motor_move(LOWER_INTAKE_PORT, -1*intake_speed);
        motor_move(MIDDLE_INTAKE_PORT, intake_speed);
        motor_move(BACK_INTAKE_PORT, intake_speed);
      } else if (!main_intake_reversed) {
        motor_move(LOWER_INTAKE_PORT, intake_speed);
        motor_move(MIDDLE_INTAKE_PORT, -1*intake_speed);

        // control the back intake roller separately from the rest of the intake so that the intake can be going forward, moving the balls up and through while at the same time the back intake roller can spin reversed to score on the center goal if desired since they're lower than the long goals.
        if (!back_intake_roller_reversed) motor_move(BACK_INTAKE_PORT, -1*intake_speed);
        else if (back_intake_roller_reversed) motor_move(BACK_INTAKE_PORT, intake_speed);
      }

    } else if(!intake_power) {
      motor_move(LOWER_INTAKE_PORT, 0);
      motor_move(MIDDLE_INTAKE_PORT, 0);
      motor_move(BACK_INTAKE_PORT, 0);
    }
}


// Chassis constructor
ez::Drive chassis(
    {LEFT_FRONT_PORT, LEFT_MIDDLE_PORT, LEFT_REAR_PORT},     // Left Chassis Ports
    {RIGHT_FRONT_PORT, RIGHT_MIDDLE_PORT, RIGHT_REAR_PORT},  // Right Chassis Ports

    INERTIAL_SENSOR,      // IMU Port
    2.8125, // wheel diameter (remember slightly bigger than what vex says
    450);   // Wheel RPM = cartridge * (motor gear / wheel gear)

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
    ez::tracking_wheel vert_tracker(TRACKING_WHEEL_ROTATIONAL_SENSOR, 2.0625, 0);   // This tracking wheel is parallel to the drive wheels

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  //ez::ez_template_print();
  pros::delay(500);  // Stop the user from doing anything while legacy ports configure
  adi_port_set_config(MATCH_LOADER, E_ADI_DIGITAL_OUT);
  adi_port_set_config(HOOD, E_ADI_DIGITAL_OUT);
  adi_port_set_config(DESCORE_MECH, E_ADI_DIGITAL_OUT);
  if (usd_is_installed() == 0) {
    controller_print(E_CONTROLLER_MASTER, 0, 0, "No uSD card!");
  } else if (!competition_is_connected() && usd_is_installed() == 1) {
    controller_print(E_CONTROLLER_MASTER, 0, 0, "..");
  }
  adi_digital_write(MATCH_LOADER, false);

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  // chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker

   chassis.odom_tracker_left_set(&vert_tracker);
   // Our tracking wheel is basically perfectly centered so I'm not actually that worried about doing this

  // Configure your chassis controls
  // chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  // chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  // chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      //{"Mogo side: DONT USE UNTESTED right (as red)\n\nAuton three rings\n", right_red_auton_three_rings},
      //{"Mogo side: DONT USE UNTESTED left (as blue)\n\nAuton three rings\n", left_blue_auton_three_rings},
      /*{"Ring side: left (as red)\n\nAuton four rings\n", left_red_auton_four_rings},
      {"Ring side: right (as blue)\n\nAuton four rings\n", right_blue_auton_four_rings},
      {"Skills", skills},
      {"Left blue score on the alliance stake", left_blue_auton_alliance_stake},
      {"right red score on the alliance stake", right_red_auton_alliance_stake},
      {"go backward off the line", goofftheline},
      {"left blue score three rings\n\nnot on the alliance stake\nposition same as the one that does\n", left_blue_auton_no_alliance_stake},
      {"right red score three rings\n\nnot on the alliance stake\nposition same as the one that does\n", right_red_auton_no_alliance_stake}*/

      {"right side testing", right_auto},
      {"left side testing", left_auto}
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

void competition_initialize() {}

void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

void opcontrol() {
  chassis.drive_mode_set(ez::DISABLE); // disable the PID after autonomous is done

  chassis.drive_brake_set(MOTOR_BRAKE_COAST); // This is preference to what you like to drive on


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
    if (count % 10 == 0) {
        controller_print(E_CONTROLLER_MASTER, 1, 0, "brain: %f", battery_get_capacity());
    } else if (count % 10 == 5) {
      if (hood_extended) { controller_print(E_CONTROLLER_MASTER, 2, 0, "hood shut");
      } else if (!hood_extended) { controller_print(E_CONTROLLER_MASTER, 2, 0, "hood open"); }
    }

    count++;

    delay(10);
  }
}
