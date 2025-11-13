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

  chassis.pid_turn_set(25_deg, TURN_SPEED); // go kind of towards the corner of the field 
  chassis.pid_wait();
  chassis.pid_drive_set(28.5_in, DRIVE_SPEED, true); 
  chassis.pid_wait();
  set_intake(false, false, false); // turn off the intake
  chassis.pid_turn_set(90_deg, TURN_SPEED); // turn to the right angle so the robot can back up into the goal
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
  adi_digital_write(HOOD, true); // close the hood
  set_intake(true, false, false); // intake on
  chassis.pid_drive_set(29_in, 70, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-1.25_in, 50, true); // move back out of the tube a little to get the balls in the right spot
  chassis.pid_wait();
  chassis.pid_drive_set(0.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  delay(370);
  chassis.pid_drive_set(-4_in, 127, true); // go away from the match loader tube
  chassis.pid_wait();
  set_intake(false, false, false); // intake off
  chassis.pid_turn_set(225_deg, TURN_SPEED); // turn toward the lower goal
  adi_digital_write(MATCH_LOADER, false);
  chassis.pid_wait();
  chassis.pid_drive_set(54_in, DRIVE_SPEED, true); // go to lower goal
  chassis.pid_wait();
  set_intake(true, true, true, 80); // intake all spins backward but slower so it doesn't go all the way through the bottom goal
}
/*
void right_red_auton_three_rings() {
  // release the front intake part by unhooking the rubber band
  toggle_intake(true, true);
  delay(200);
  toggle_intake(false, false);
  
  adi_digital_write(MOGO, false);
  chassis.pid_drive_set(-20.5_in, DRIVE_SPEED, true); // back up
  chassis.pid_wait();
  chassis.pid_turn_set(-30_deg, TURN_SPEED); // Turn to the mogo
  chassis.pid_wait();
  chassis.pid_drive_set(-16_in, 80, true); // back up into the mogo.
  chassis.pid_wait();
  chassis.pid_drive_set(-8_in, 80, true); // keep backing up the mogo while clamping onto it.
  delay(90);
  adi_digital_write(MOGO, true); // clamp on the mogo while slowly moving
  chassis.pid_wait();

  chassis.pid_turn_set(-72_deg, TURN_SPEED); // Turn towards the rings
  chassis.pid_wait();
  toggle_intake(true, false);

  chassis.pid_drive_set(28_in, DRIVE_SPEED, true); // move toward the rings
  chassis.pid_wait();
  toggle_intake(true, true); // briefly reverse the intake so the hook can't get stuck
  delay(100);
  toggle_intake(true, false);

  delay(650); // make sure it has some time to pick up the ring before turning again
  chassis.pid_turn_set(-25_deg, TURN_SPEED);
  toggle_intake(true, true); // briefly reverse the intake so the hook can't get stuck
  delay(100);
  toggle_intake(true, false);
  chassis.pid_wait();
  chassis.pid_drive_set(20_in, DRIVE_SPEED, false); // move towards the four rings in the corner
  chassis.pid_wait();
  chassis.pid_turn_set(5_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(10_in, DRIVE_SPEED, false); // move towards the four rings in the corner
  chassis.pid_wait();
  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(14_in, 127, true); // pickup the bottom ring
  chassis.pid_wait();
  delay(2000);
  toggle_intake(true, false);
  chassis.pid_drive_set(-15_in, 15, true);
  chassis.pid_wait();
  chassis.pid_turn_set(153_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(47_in, DRIVE_SPEED, true); // go to ladder
  chassis.pid_wait();
  adi_digital_write(DOINKER, true);
  chassis.drive_mode_set(ez::DISABLE); // disable the PID after autonomous is done


}void left_blue_auton_three_rings() {
  // release the front intake part by unhooking the rubber band
  toggle_intake(true, true);
  delay(200);
  toggle_intake(false, false);

  adi_digital_write(MOGO, false);
  chassis.pid_drive_set(-20.5_in, DRIVE_SPEED, true); // back up
  chassis.pid_wait();
  chassis.pid_turn_set(30_deg, TURN_SPEED); // Turn to the mogo
  chassis.pid_wait();
  chassis.pid_drive_set(-16_in, 80, true); // back up into the mogo.
  chassis.pid_wait();
  chassis.pid_drive_set(-8_in, 80, true); // keep backing up the mogo while clamping onto it.
  delay(90);
  adi_digital_write(MOGO, true); // clamp on the mogo while slowly moving
  chassis.pid_wait();

  chassis.pid_turn_set(73_deg, TURN_SPEED); // Turn towards the rings
  chassis.pid_wait();
  toggle_intake(true, false);

  chassis.pid_drive_set(28_in, DRIVE_SPEED, true); // move toward the rings
  chassis.pid_wait();
  toggle_intake(true, true); // briefly reverse the intake so the hook can't get stuck
  delay(100);
  toggle_intake(true, false);
  delay(650); // make sure it has some time to pick up the ring before turning again
  chassis.pid_turn_set(30_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(20_in, DRIVE_SPEED, false); // move towards the four rings in the corner chassis.pid_wait();
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(12_in, DRIVE_SPEED, false); // move towards the four rings in the corner
  chassis.pid_wait();
  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(13_in, 127, true); // pickup the bottom ring
  chassis.pid_wait();
  uint16_t i = 0;
  while(!adi_digital_read(LIMIT) && i < 3200) { i+=10; delay(10); if(i < 1200 && i > 1180) {chassis.pid_drive_set(-1_in, DRIVE_SPEED, false);chassis.pid_wait();}}
  chassis.pid_drive_set(-15_in, 15, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-146_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(47_in, DRIVE_SPEED, true); // go to ladder
  chassis.pid_wait();
  adi_digital_write(DOINKER, true);
  chassis.drive_mode_set(ez::DISABLE); // disable the PID after autonomous is done
}*//*void left_red_auton_four_rings() {
  // release the front intake part by unhooking the rubber band
  toggle_intake(true, true);
  delay(200);
  toggle_intake(false, false);

  chassis.pid_drive_set(-20.5_in, DRIVE_SPEED, true); // back up
  chassis.pid_wait();
  adi_digital_write(MOGO, false);
  chassis.pid_turn_set(30_deg, TURN_SPEED); // Turn to the mogo
  chassis.pid_wait();
  chassis.pid_drive_set(-16_in, 80, true); // back up into the mogo.
  chassis.pid_wait();
  chassis.pid_drive_set(-8_in, 80, true); // keep backing up the mogo while clamping onto it.
  delay(90);
  adi_digital_write(MOGO, true); // clamp on the mogo while slowly moving
  chassis.pid_wait();
  chassis.pid_drive_set(9.5_in, DRIVE_SPEED, true); // keep backing up the mogo while clamping onto it.
  delay(100);
  toggle_intake(true, false);
  chassis.pid_wait();
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(18.5_in, DRIVE_SPEED, true); // score the first ring, the closer ring to the ladder of the rings in the middle on the bottom
  delay(100);
  toggle_intake(true, true); // briefly reverse the intake so the hook can't get stuck
  delay(100);
  toggle_intake(true, false);
  chassis.pid_wait();
  delay(800);
  chassis.pid_drive_set(-20.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(70_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(10_in, 40, true);
  delay(150);
  toggle_intake(true, true); // briefly reverse the intake so the hook can't get stuck
  delay(100);
  toggle_intake(true, false);
  chassis.pid_wait();
  chassis.pid_turn_set(125_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  toggle_intake(true, true); // briefly reverse the intake so the hook can't get stuck
  delay(100);
  toggle_intake(true, false);
  chassis.pid_turn_set(165_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(7_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(215_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(13_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  adi_digital_write(DOINKER2, true);
  chassis.drive_mode_set(ez::DISABLE); // disable the PID after autonomous is done
}void right_blue_auton_four_rings() {
  // release the front intake part by unhooking the rubber band
  toggle_intake(true, true);
  delay(200);
  toggle_intake(false, false);

  chassis.pid_drive_set(-20.5_in, DRIVE_SPEED, true); // back up
  chassis.pid_wait();
  adi_digital_write(MOGO, false);
  chassis.pid_turn_set(-30_deg, TURN_SPEED); // Turn to the mogo
  chassis.pid_wait();
  chassis.pid_drive_set(-16_in, 80, true); // back up into the mogo.
  chassis.pid_wait();
  chassis.pid_drive_set(-8_in, 80, true); // keep backing up the mogo while clamping onto it.
  delay(90);
  adi_digital_write(MOGO, true); // clamp on the mogo while slowly moving
  chassis.pid_wait();
  chassis.pid_drive_set(9.5_in, DRIVE_SPEED, true); // keep backing up the mogo while clamping onto it.
  delay(100);
  toggle_intake(true, false);
  chassis.pid_wait();
  chassis.pid_turn_set(-135_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(18.5_in, DRIVE_SPEED, true); // score the first ring, the closer ring to the ladder of the rings in the middle on the bottom
  delay(100);
  toggle_intake(true, true); // briefly reverse the intake so the hook can't get stuck
  delay(100);
  toggle_intake(true, false);
  chassis.pid_wait();
  delay(800);
  chassis.pid_drive_set(-20.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-70_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(10_in, 40, true);
  delay(150);
  toggle_intake(true, true); // briefly reverse the intake so the hook can't get stuck
  delay(100);
  toggle_intake(true, false);
  chassis.pid_wait();
  chassis.pid_turn_set(-125_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  toggle_intake(true, true); // briefly reverse the intake so the hook can't get stuck
  delay(100);
  toggle_intake(true, false);
  chassis.pid_turn_set(-165_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(7_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-215_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(13_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  adi_digital_write(DOINKER, true);
  chassis.drive_mode_set(ez::DISABLE); // disable the PID after autonomous is done
} void skills() {
  // release the front intake part by unhooking the rubber band
  toggle_intake(true, true);
  delay(200);
  toggle_intake(false, false);

  adi_digital_write(MOGO, true);
  chassis.pid_drive_set(-16_in, 80, true); // back up into the mogo.
  chassis.pid_wait();
  chassis.pid_drive_set(-8_in, 80, true); // keep backing up the mogo while clamping onto it.
  delay(90);
  adi_digital_write(MOGO, false); // clamp on the mogo while slowly moving
  chassis.pid_wait();
  chassis.pid_drive_set(9.5_in, DRIVE_SPEED, true); // keep backing up the mogo while clamping onto it.
  delay(100);
  toggle_intake(true, false);
  chassis.pid_wait();
  chassis.pid_turn_set(-302_deg, TURN_SPEED); // turn towards the first ring
  chassis.pid_wait();
  toggle_intake(true, true); // briefly reverse the intake so the hook can't get stuck
  delay(100);
  toggle_intake(true, false);
  chassis.pid_drive_set(30_in, 25, true);
  delay(200);
  toggle_intake(true, true); // briefly reverse the intake so the hook can't get stuck
  delay(100);
  toggle_intake(true, false);
  chassis.pid_wait();
  delay(400);
  chassis.pid_turn_set(524_deg, TURN_SPEED); // turn towards the third ring
  chassis.pid_wait();
  chassis.pid_drive_set(15.2_in, 40, true); // get the third ring (go slow because the second ring takes time to intake
  delay(100);
  toggle_intake(true, true); // briefly reverse the intake so the hook can't get stuck
  delay(100);
  toggle_intake(true, false);
  chassis.pid_wait();
  chassis.pid_turn_set(215_deg, 127); // turn towards the fourth ring
  chassis.pid_wait();
  chassis.pid_drive_set(15_in, 60, true); // get the fourth
  delay(100);
  toggle_intake(true, true); // briefly reverse the intake so the hook can't get stuck
  delay(100);
  toggle_intake(true, false);
  chassis.pid_wait();
  chassis.pid_drive_set(15_in, DRIVE_SPEED, true); // get the fourth
  chassis.pid_wait();
  chassis.pid_turn_set(195.5_deg, TURN_SPEED); // turn towards the corner
  chassis.pid_wait();
  chassis.pid_drive_set(-52.5_in, DRIVE_SPEED, true); // used to be 51"
  toggle_intake(true, true); // briefly reverse the intake so the hook can't get stuck
  delay(100);
  toggle_intake(true, false);
  chassis.pid_wait();
  adi_digital_write(MOGO, true); // release the first mogo in the corner
  chassis.pid_drive_set(16.5_in, DRIVE_SPEED, true); // used to be 15 inches
  chassis.pid_wait();
  chassis.pid_turn_set(60.0_deg, TURN_SPEED); // turn towards the other corner on the starting side
  chassis.pid_wait();
  chassis.pid_drive_set(-68.0_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(70_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(85_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-33_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-30_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-50_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-55_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-60_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(35_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-15_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-35_in, DRIVE_SPEED, true); // score the third mogo in the corner
  chassis.pid_wait();

  // score the fourth ring? 
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(61.5_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(110_in, DRIVE_SPEED, true);
  chassis.pid_wait();
} void left_blue_auton_alliance_stake() {
  for(int i = 0; i < 1000; i+=10) {
    move_lb(1300);
    delay(10);
  }
  for(int i = 0; i < 600; i+=10) {
    move_lb(-120);
    delay(10);
  }
  motor_tare_position(RIGHT_LB_MOTOR); // zero it out to the original position
  chassis.pid_drive_set(-28_in, 60, true); // move is back slow so it doesn't bump the wrong color ring into the future path
  chassis.pid_wait();
  chassis.pid_turn_set(85_deg, 60);
  chassis.pid_wait();
  chassis.pid_drive_set(-17_in, 70, true);
  chassis.pid_wait();

  // clamp the mobile goal
  chassis.pid_drive_set(-19_in, 60, true);
  delay(350);
  adi_digital_write(MOGO, true);
  chassis.pid_wait();
  chassis.pid_drive_set(10_in, 127, true);
  chassis.pid_wait();

  // get ring on mobile goal then start moving toward the corner
  toggle_intake(true, false);
  chassis.pid_turn_set(135_deg, 110);
  chassis.pid_wait();
  chassis.pid_drive_set(28_in, 127, true); // grab the ring
  chassis.pid_wait();
  delay(500);
  chassis.pid_turn_set(85_deg, 100);
  chassis.pid_wait();
  chassis.pid_drive_set(19_in, 127, true);
  chassis.pid_wait();

  chassis.pid_turn_set(30_deg, 100);
  chassis.pid_wait();
  chassis.pid_drive_set(14_in, 100, true);
  toggle_intake(false, false);
  chassis.pid_wait();
  chassis.pid_turn_set(100_deg, 100);
  toggle_intake(true, false);
  chassis.pid_wait();
  chassis.pid_drive_set(18_in, 50, true); // grab the bottom ring out of the corner
  chassis.pid_wait();
  chassis.pid_drive_set(-17_in, 40, true); // back out of the corner slowly
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, 55);
  chassis.pid_wait();
  chassis.pid_drive_set(5_in, 127, true);
  chassis.pid_wait();
} void right_red_auton_alliance_stake() {
  delay(3000);
  for(int i = 0; i < 1000; i+=10) {
    move_lb(1300);
    delay(10);
  }
  for(int i = 0; i < 600; i+=10) {
    move_lb(-120);
    delay(10);
  }
  motor_tare_position(RIGHT_LB_MOTOR); // zero it out to the original position
  chassis.pid_drive_set(-28_in, 60, true); // move is back slow so it doesn't bump the wrong color ring into the future path
  chassis.pid_wait();
  chassis.pid_turn_set(-85_deg, 60);
  chassis.pid_wait();
  chassis.pid_drive_set(-17_in, 70, true);
  chassis.pid_wait();

  // clamp the mobile goal
  chassis.pid_drive_set(-19_in, 60, true);
  delay(350);
  adi_digital_write(MOGO, true);
  chassis.pid_wait();
  chassis.pid_drive_set(10_in, 127, true);
  chassis.pid_wait();

  // get ring on mobile goal then start moving toward the corner
  toggle_intake(true, false);
  chassis.pid_turn_set(-135_deg, 110);
  chassis.pid_wait();
  chassis.pid_drive_set(28_in, 127, true); // grab the ring
  chassis.pid_wait();
  delay(500);
  chassis.pid_turn_set(-85_deg, 100);
  chassis.pid_wait();
  chassis.pid_drive_set(19_in, 127, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-30_deg, 100);
  chassis.pid_wait();
  chassis.pid_drive_set(14_in, 100, true);
  toggle_intake(false, false);
  chassis.pid_wait();
  chassis.pid_turn_set(-100_deg, 100);
  toggle_intake(true, false);
  chassis.pid_wait();
  chassis.pid_drive_set(18_in, 50, true); // grab the bottom ring out of the corner
  chassis.pid_wait();
  chassis.pid_drive_set(-17_in, 40, true); // back out of the corner slowly
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, 55);
  chassis.pid_wait();
  chassis.pid_drive_set(5_in, 127, true);
  chassis.pid_wait();
} void goofftheline() {
chassis.pid_drive_set(-10_in, 127, false);
} void right_red_auton_no_alliance_stake() {
  chassis.pid_drive_set(-28_in, 60, true); // move is back slow so it doesn't bump the wrong color ring into the future path
  chassis.pid_wait();
  chassis.pid_turn_set(-85_deg, 60);
  chassis.pid_wait();
  chassis.pid_drive_set(-17_in, 70, true);
  chassis.pid_wait();

  // clamp the mobile goal
  chassis.pid_drive_set(-19_in, 60, true);
  delay(350);
  toggle_intake(true, false);
  adi_digital_write(MOGO, true);
  chassis.pid_wait();
  chassis.pid_drive_set(10_in, 127, true);
  chassis.pid_wait();

  // get ring on mobile goal then start moving toward the corner
  chassis.pid_turn_set(-135_deg, 110);
  chassis.pid_wait();
  chassis.pid_drive_set(28_in, 127, true); // grab the ring
  chassis.pid_wait();
  delay(500);
  chassis.pid_turn_set(-85_deg, 100);
  chassis.pid_wait();
  chassis.pid_drive_set(19_in, 127, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-30_deg, 100);
  chassis.pid_wait();
  chassis.pid_drive_set(14_in, 100, true);
  toggle_intake(false, false);
  chassis.pid_wait();
  chassis.pid_turn_set(-100_deg, 100);
  toggle_intake(true, false);
  chassis.pid_wait();
  chassis.pid_drive_set(18_in, 50, true); // grab the bottom ring out of the corner
  chassis.pid_wait();
  chassis.pid_drive_set(-17_in, 40, true); // back out of the corner slowly
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, 55);
  chassis.pid_wait();
  chassis.pid_drive_set(5_in, 127, true);
  chassis.pid_wait();

} void left_blue_auton_no_alliance_stake() {
  chassis.pid_drive_set(-28_in, 60, true); // move is back slow so it doesn't bump the wrong color ring into the future path
  chassis.pid_wait();
  chassis.pid_turn_set(85_deg, 60);
  chassis.pid_wait();
  chassis.pid_drive_set(-17_in, 70, true);
  chassis.pid_wait();

  // clamp the mobile goal
  chassis.pid_drive_set(-19_in, 60, true);
  delay(350);
  toggle_intake(true, false);
  adi_digital_write(MOGO, true);
  chassis.pid_wait();
  chassis.pid_drive_set(10_in, 127, true);
  chassis.pid_wait();

  // get ring on mobile goal then start moving toward the corner
  chassis.pid_turn_set(135_deg, 110);
  chassis.pid_wait();
  chassis.pid_drive_set(28_in, 127, true); // grab the ring
  chassis.pid_wait();
  delay(500);
  chassis.pid_turn_set(85_deg, 100);
  chassis.pid_wait();
  chassis.pid_drive_set(19_in, 127, true);
  chassis.pid_wait();

  chassis.pid_turn_set(30_deg, 100);
  chassis.pid_wait();
  chassis.pid_drive_set(14_in, 100, true);
  toggle_intake(false, false);
  chassis.pid_wait();
  chassis.pid_turn_set(100_deg, 100);
  toggle_intake(true, false);
  chassis.pid_wait();
  chassis.pid_drive_set(18_in, 50, true); // grab the bottom ring out of the corner
  chassis.pid_wait();
  chassis.pid_drive_set(-17_in, 40, true); // back out of the corner slowly
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, 55);
  chassis.pid_wait();
  chassis.pid_drive_set(5_in, 127, true);
  chassis.pid_wait();
}*/

