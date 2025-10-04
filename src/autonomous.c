#include <math.h>
#include "main.h"

/* -------------------------------------------------------------------------------------------- */
// The IMU and tracking wheel rotation sensor ports are currently hardcoded to 4 & 9, respectively
/* -------------------------------------------------------------------------------------------- */

// constants used for turning
double t_kP = 1.25;
double t_kD = 7;
double t_kI = 0.2;

// constants used for driving 
double d_kP = 0.15;
double d_kD = 1.25;
double d_kI = 0.017;


double error = 5; //So it goes through the loop the first time
double derivative;
double prevError;
double integral;
double power;

void turn(uint8_t LEFT_FRONT_PORT, uint8_t LEFT_REAR_PORT, uint8_t LEFT_MIDDLE_PORT, uint8_t RIGHT_FRONT_PORT, uint8_t RIGHT_REAR_PORT, uint8_t RIGHT_MIDDLE_PORT, double target_degrees) {
  int temp = 0;
  integral = 0;
  derivative = 0;
  while (fabs(error) > 3.5 || fabs(derivative) > 0.02) {
    error = target_degrees - imu_get_rotation(4);
    derivative = error - prevError;
    integral += error;
    if ((error < 0 && prevError > 0) || (error < 0 && prevError > 0)) integral = 0; /* when error crosses zero, reset inegral */
    if (integral*t_kI > 55) { integral = 55; }
    power = -1*(error*t_kP + derivative*t_kD + integral*t_kI);
    if (power > 127) {power = 127;}
    if (power < -127) {power = -127;}
    motor_move(LEFT_FRONT_PORT, power);
    motor_move(LEFT_REAR_PORT, power);
    motor_move(LEFT_MIDDLE_PORT, power);
    motor_move(RIGHT_FRONT_PORT, power);
    motor_move(RIGHT_REAR_PORT, power);
    motor_move(RIGHT_MIDDLE_PORT, power);
    prevError = error;
    delay(15);
    temp ++;
    if (temp % 5 == 0) {
      controller_print(E_CONTROLLER_MASTER, 0, 0, "%lf", error);
    }
  }
  power = 0;
  motor_move(LEFT_FRONT_PORT, power);
  motor_move(LEFT_REAR_PORT, power);
  motor_move(LEFT_MIDDLE_PORT, power);
  motor_move(RIGHT_FRONT_PORT, power);
  motor_move(RIGHT_REAR_PORT, power);
  motor_move(RIGHT_MIDDLE_PORT, power);
  error = 5; // just so it'll go through the next task at first
}

void move(uint8_t LEFT_FRONT_PORT, uint8_t LEFT_REAR_PORT, uint8_t LEFT_MIDDLE_PORT, uint8_t RIGHT_FRONT_PORT, uint8_t RIGHT_REAR_PORT, uint8_t RIGHT_MIDDLE_PORT, double target_degrees, bool reversed, bool ignore_overshoot) { // ignore overshoot is in place for things like picking up the mogo, where it's better to just accept a little bit of overshoot occasionally than risk overshooting, moving the mogo too far away, then going back to where it should have gone to and now being too far away from the mogo to be able to pick it up.
  int temp = 0;
  integral = 0;
  derivative = 0;
  rotation_reset(9);
  rotation_set_reversed(9, !reversed);
  while (fabs(error) > 2.99 || fabs(derivative) > 0.03) {
    error = target_degrees - rotation_get_position(9) / 100.0f; // centidegrees to degrees
    derivative = error - prevError;
    integral += error;
    if (integral*d_kI > 46) { integral = 46; }
    power = error*d_kP + derivative*d_kD + integral*d_kI;
    if (power > 127) {power = 127;}
    if (power < -127) {power = -127;}
    if (!reversed) {
      motor_move(-1*LEFT_FRONT_PORT, power);
      motor_move(-1*LEFT_REAR_PORT, power);
      motor_move(-1*LEFT_MIDDLE_PORT, power);
      motor_move(RIGHT_FRONT_PORT, power);
      motor_move(RIGHT_REAR_PORT, power);
      motor_move(RIGHT_MIDDLE_PORT, power);
    } else if (reversed) {
      motor_move(LEFT_FRONT_PORT, power);
      motor_move(LEFT_REAR_PORT, power);
      motor_move(LEFT_MIDDLE_PORT, power);
      motor_move(-1*RIGHT_FRONT_PORT, power);
      motor_move(-1*RIGHT_REAR_PORT, power);
      motor_move(-1*RIGHT_MIDDLE_PORT, power);
    }
    prevError = error;
    if (ignore_overshoot && error < 0) { break; }
    delay(15);
    temp ++;
    if (temp % 5 == 0) {
      controller_print(E_CONTROLLER_MASTER, 0, 0, "%lf", error);
    }
  }
  power = 0;
  motor_move(-1*LEFT_FRONT_PORT, power);
  motor_move(-1*LEFT_REAR_PORT, power);
  motor_move(-1*LEFT_MIDDLE_PORT, power);
  motor_move(RIGHT_FRONT_PORT, power);
  motor_move(RIGHT_REAR_PORT, power);
  motor_move(RIGHT_MIDDLE_PORT, power);
  error = 5;

}
