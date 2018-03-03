#ifndef SRC_MANIPULATOR_H_
#define SRC_MANIPULATOR_H_

#include <Constants.h>
#include <Safety.h>

class Manipulator
{
	public:
		Manipulator(Joystick *controller, Safety *safe);
		void update();
		void reset();

	protected:

	private:
		// Maximum joint angle velocity (in degrees per second times manipulatorPeriod in seconds/cycle)
		const float maxSpeed = 50.0 * ((float)manipulatorPeriod / 1000000.0);

		// Maximum joint angle acceleration (in degrees per second squared times manipulatorPeriod in seconds/cycle squared)
		const float maxAccel = 50.0 * ((float)manipulatorPeriod / 1000000.0) * ((float)manipulatorPeriod / 1000000.0);

		// Proportional constants for manipulator position control (tuned for balanced acceleration and deceleration)
		const float kProportional[NUM_MANIPULATOR_JOINTS] =
		{
			0.01,
			0.015,
			0.015,
			0.01,
			0.005,
			0.005,
			0.02
		};

		// Derivative constants for manipulator position control (tuned for balanced acceleration and deceleration)
		const float kDerivative[NUM_MANIPULATOR_JOINTS] =
		{
			0.4,
			0.6,
			0.6,
			0.4,
			0.2,
			0.1,
			0.4
		};

		// Maximum step by which the motor controller power can change by per cycle
		const float powerChangeMax = 0.03;

		// Maximum current value, upper and lower bounds, adjusted by throttle
		const float maxCurrentUpper = 15;
		const float maxCurrentLower = 10;

		std::shared_ptr<Victor> motorControllers[NUM_MANIPULATOR_JOINTS];
		std::shared_ptr<AnalogPotentiometer> potentiometers[NUM_MANIPULATOR_JOINTS];

		float destPosition[NUM_MANIPULATOR_JOINTS];
		float trackPosition[NUM_MANIPULATOR_JOINTS];

		float lastSpeed[NUM_MANIPULATOR_JOINTS];
		float lastPower[NUM_MANIPULATOR_JOINTS];
		float lastError[NUM_MANIPULATOR_JOINTS];
		float capPower[NUM_MANIPULATOR_MOTORS];

		uint32_t lastRunTimestamp;
		Joystick *joystick;
		Safety *safety;
};

#endif /* SRC_MANIPULATOR_H_ */
