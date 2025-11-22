#include "main.h"

#define LOWER_INTAKE_PORT 20
#define MIDDLE_INTAKE_PORT 2
#define BACK_INTAKE_PORT 3 

#define INERTIAL_SENSOR 4
#define TRACKING_WHEEL_ROTATIONAL_SENSOR 9

#define MATCH_LOADER 'H'
#define HOOD 'A'
#define DESCORE_MECH 'B' // descores the balls from the long goals

using namespace pros::c;

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(28.0, 1.0, 95.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there

  // change this later to the actual degree value
}
void right_auto() {
  adi_digital_write(HOOD, true);
  chassis.pid_drive_set(8_in, DRIVE_SPEED, true); // go forwards (right from driver's POV) away from the park area
  chassis.pid_wait();
  chassis.pid_turn_set(-89_deg, TURN_SPEED); // turn towards the three balls
  chassis.pid_wait();
  set_intake(true, false, false);
  chassis.pid_drive_set(21_in, DRIVE_SPEED-10, true); // go toward the three balls
  chassis.pid_wait();
  chassis.pid_drive_set(15_in, 40, true); // slow down to get the balls easier
  chassis.pid_wait(); // pick up the three balls

  chassis.pid_turn_set(31_deg, TURN_SPEED); // go kind of towards the corner of the field 
  chassis.pid_wait();
  chassis.pid_drive_set(27.5_in, 120, true); 
  chassis.pid_wait();
  set_intake(false, false, false); // turn off the intake
  chassis.pid_turn_set(90_deg, TURN_SPEED); // turn to the right angle so the robot can back up into the goal
  chassis.pid_wait();
  chassis.pid_drive_set(-11.5_in, DRIVE_SPEED, true); // back up into the goal
  chassis.pid_wait();
  set_intake(true, false, false); // turn on the intake
  adi_digital_write(HOOD, false); // open the hood so balls can come out
  delay(1700); // wait a little bit and then all the balls will be scored
  
  // Drive toward the match loader with the piston out
  adi_digital_write(MATCH_LOADER, true);
  set_intake(false, false, false);
  delay(150);

  chassis.pid_drive_set(28.5_in, 70, true); // drive away from the long goal
  chassis.pid_wait();
  delay(150);

  adi_digital_write(HOOD, true); // close the hood
  set_intake(true, false, false); // intake on
  chassis.pid_drive_set(-1.25_in, 50, true); // move back out of the tube a little to get the balls in the right spot
  chassis.pid_wait();
  chassis.pid_drive_set(0.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  delay(370);
  chassis.pid_drive_set(-7_in, 127, true); // go away from the match loader tube
  chassis.pid_wait();
  set_intake(false, false, false); // intake off
  chassis.pid_turn_set(225_deg, TURN_SPEED); // turn toward the lower goal
  adi_digital_write(MATCH_LOADER, false);
  chassis.pid_wait();
  chassis.pid_drive_set(52_in, DRIVE_SPEED, true); // go to lower goal
  chassis.pid_wait();
  set_intake(true, true, true, 120); // intake all spins backward but slower so it doesn't go all the way through the bottom goal
} void left_auto() {
  adi_digital_write(HOOD, true);
  chassis.pid_drive_set(8_in, DRIVE_SPEED, true); // go forwards (left from driver's POV) away from the park area
  chassis.pid_wait();
  chassis.pid_turn_set(89_deg, TURN_SPEED); // turn towards the three balls
  chassis.pid_wait();
  set_intake(true, false, false);
  chassis.pid_drive_set(21_in, DRIVE_SPEED-10, true); // go toward the three balls
  chassis.pid_wait();
  chassis.pid_drive_set(15_in, 40, true); // slow down to get the balls easier
  chassis.pid_wait(); // pick up the three balls

  chassis.pid_turn_set(-31_deg, TURN_SPEED); // go kind of towards the corner of the field 
  chassis.pid_wait();
  chassis.pid_drive_set(26.8_in, 120, true); 
  chassis.pid_wait();
  set_intake(false, false, false); // turn off the intake
  chassis.pid_turn_set(-90_deg, TURN_SPEED); // turn to the right angle so the robot can back up into the goal
  chassis.pid_wait();
  chassis.pid_drive_set(-11_in, DRIVE_SPEED, true); // back up into the goal
  chassis.pid_wait();
  set_intake(true, false, false); // turn on the intake
  adi_digital_write(HOOD, false); // open the hood so balls can come out
  delay(1700); // wait a little bit and then all the balls will be scored
  
  // Drive toward the match loader with the piston out
  adi_digital_write(MATCH_LOADER, true);
  set_intake(false, false, false);
  delay(150);

  chassis.pid_drive_set(28.5_in, 70, true); // drive away from long goal
  delay(150);
  adi_digital_write(HOOD, true); // close the hood
  set_intake(true, false, false); // intake on
  chassis.pid_wait();
  chassis.pid_drive_set(-1.25_in, 50, true); // move back out of the tube a little to get the balls in the right spot
  chassis.pid_wait();
  chassis.pid_drive_set(0.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  delay(360);
  chassis.pid_drive_set(-7_in, 127, true); // go away from the match loader tube
  chassis.pid_wait();
  set_intake(false, false, false); // intake off
  chassis.pid_turn_set(-46_deg, TURN_SPEED); // turn toward the middle goal
  adi_digital_write(MATCH_LOADER, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-52.5_in, 85, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-0.75_in, 45, true); // back slowly up
  chassis.pid_wait();
  set_intake(true, false, true);
} void move_forward_an_inch() {
  chassis.pid_drive_set(1_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}
