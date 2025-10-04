#include <math.h>
#include "drive.h"
#include "main.h"
// Curvature Drive but slightly modified to work how I need
// https://wiki.purduesigbots.com/software/robotics-basics/curvature-cheesy-drive

// Cheesy Drive Constants
#define DRIVE_DEADBAND 0.1f
#define DRIVE_SLEW 0.02f
#define CD_TURN_NONLINEARITY 0.65 // This factor determines how fast the wheel traverses the "non linear" sine curve
#define CD_NEG_INERTIA_SCALAR 4.0
#define CD_SENSITIVITY 1.0


// We apply a sinusoidal curve (twice) to the joystick input to give finer
// control at small inputs.
static double _turnRemapping(double iturn) {
	double denominator = sin(M_PI_2 * CD_TURN_NONLINEARITY);
	double firstRemapIteration = sin(M_PI_2 * CD_TURN_NONLINEARITY * iturn) / denominator;
	return sin(M_PI_2 * CD_TURN_NONLINEARITY * firstRemapIteration) / denominator;
}

// On each iteration of the drive controller (where we aren't point turning) we
// constrain the accumulators to the range [-1, 1].
double quickStopAccumlator = 0.0;
double negInertiaAccumlator = 0.0;
static void _updateAccumulators() {
	if (negInertiaAccumlator > 1) { negInertiaAccumlator -= 1; }
	else if (negInertiaAccumlator < -1) { negInertiaAccumlator += 1; }
	else { negInertiaAccumlator = 0; }

	if (quickStopAccumlator > 1) { quickStopAccumlator -= 1; }
        else if (quickStopAccumlator < -1) { quickStopAccumlator += 1; }
	else { quickStopAccumlator = 0.0; }
}

double left, right;

double prevTurn = 0.0;
double prevThrottle = 0.0;
void cheesyDrive(double ithrottle, double iturn) {
	bool turnInPlace = false;
	double linearCmd = ithrottle;
	if (fabs(ithrottle) < DRIVE_DEADBAND && fabs(iturn) > DRIVE_DEADBAND) {
		// The controller joysticks can output values near zero when they are
		// not actually pressed. In the case of small inputs like this, we
		// override the throttle value to 0.
		linearCmd = 0.0;
		turnInPlace = true;
	} else if (ithrottle - prevThrottle > DRIVE_SLEW) {
		linearCmd = prevThrottle + DRIVE_SLEW;
	} else if (ithrottle - prevThrottle < -(DRIVE_SLEW * 2)) {
		// We double the drive slew rate for the reverse direction to get
		// faster stopping.
		linearCmd = prevThrottle - (DRIVE_SLEW * 2);
	}

	double remappedTurn = _turnRemapping(iturn);

	if (turnInPlace) {
		// The remappedTurn value is squared when turning in place. This
		// provides even more fine control over small speed values.
		left = remappedTurn * fabs(remappedTurn);
		right = -remappedTurn * fabs(remappedTurn);

	} else if (!turnInPlace) {
		double negInertiaPower = (iturn - prevTurn) * CD_NEG_INERTIA_SCALAR;
		negInertiaAccumlator += negInertiaPower;

		double angularCmd =
		    fabs(linearCmd) *  // the more linear vel, the faster we turn
		        (remappedTurn + negInertiaAccumlator) *
		        CD_SENSITIVITY -  // we can scale down the turning amount by a constant
		    quickStopAccumlator;

		right = left = linearCmd;
		left += angularCmd;
		right -= angularCmd;

		_updateAccumulators();
	}

	prevTurn = iturn;
	prevThrottle = ithrottle;
}

void move_drivetrain(uint8_t LEFT_FRONT_PORT, uint8_t LEFT_REAR_PORT, uint8_t LEFT_MIDDLE_PORT, uint8_t RIGHT_FRONT_PORT, uint8_t RIGHT_REAR_PORT, uint8_t RIGHT_MIDDLE_PORT, double ithrottle, double iturn) {
  cheesyDrive(ithrottle, iturn);

  motor_move(LEFT_FRONT_PORT, -127*left);
  motor_move(LEFT_REAR_PORT, 127*left);
  motor_move(LEFT_MIDDLE_PORT, -127*left);
  motor_move(RIGHT_FRONT_PORT, 127*right);
  motor_move(RIGHT_REAR_PORT, -127*right);
  motor_move(RIGHT_MIDDLE_PORT, 127*right);
}

