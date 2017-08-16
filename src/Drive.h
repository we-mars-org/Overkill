#ifndef SRC_DRIVE_H_
#define SRC_DRIVE_H_

#include <Constants.h>
#include <Safety.h>

/*
 * Encoder counts per revolution = 7
 * x4 = 28 counts/rev
 * Gear ratio = 71
 * Counts per rotation of output shaft = 28 * 71 = 1988 counts/rev
 * Wheel circumference = 0.67 m/rev
 * Counts per meter = 2970 counts/m
 *
 * Motor no load speed = 75 rpm
 * Wheel no load velocity = (75 / 60) * 0.67 m = 0.84 m/s <- MAX SPEED CAPACITY = 2500 counts/s
 * Max speed from competition rules = 3 km/hr = 0.83 m/s <- CAPPED MAX SPEED = 2400 counts/s
 */

class Drive
{
	public:
		Drive(Joystick *controller, Safety *safe);
		void update();
		void reset();

	private:
		// Maximum drive motor velocity in encoder counts per loop period (refer to calculation above)
		const float maxSpeed = (float)(2400 * drivePeriod / 1000000); // drivePeriod is in microseconds per period

		// Deadband within which the joystick axes input is ignored
		const float joystickDeadband = 0.05;

		// Integral constant for drive speed control (tuned for balanced acceleration and deceleration)
		const float kIntegral = 0.002;

		// Maximum step by which the motor controller power can change by per cycle
		const float powerChangeMax = 0.10;

		// Maximum current value, upper and lower bounds, adjusted by throttle
		const float maxCurrentUpper = 15;
		const float maxCurrentLower = 10;

		std::shared_ptr<Victor> motorControllers[NUM_DRIVE_MOTORS];
		std::shared_ptr<Encoder> encoders[NUM_DRIVE_MOTORS];

		uint32_t lastEncoder[NUM_DRIVE_MOTORS];
		float lastPower[NUM_DRIVE_MOTORS];
		float capPower[NUM_DRIVE_MOTORS];

		uint32_t lastRunTimestamp;
		Joystick *joystick;
		Safety *safety;
};

#endif /* SRC_DRIVE_H_ */
