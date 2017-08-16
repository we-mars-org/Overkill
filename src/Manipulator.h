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
		const float maxSpeed = 100.0 * ((float)manipulatorPeriod / 1000000.0);

		// Maximum joint angle acceleration (in degrees per second squared times manipulatorPeriod in seconds/cycle squared)
		const float maxAccel = 100.0 * ((float)manipulatorPeriod / 1000000.0) * ((float)manipulatorPeriod / 1000000.0);

		// Proportional constants for manipulator position control (tuned for balanced acceleration and deceleration)
		const float kProportional[NUM_MANIPULATOR_MOTORS] =
		{
			0.02,
			0.02,
			0.02,
			0.02,
			0.02,
			0.02,
			0.02
		};

		// Integral constants for manipulator position control (tuned for balanced acceleration and deceleration)
		const float kIntegral[NUM_MANIPULATOR_MOTORS] =
		{
			0.00000,
			0.00000,
			0.00000,
			0.00000,
			0.00000,
			0.00000,
			0.00000
		};

		// Integral accumulator limit to control oscillations
		const float kIntegralLimit = 0.1;

		// Maximum step by which the motor controller power can change by per cycle
		const float powerChangeMax = 0.1;

		// Deadband in degrees within which a position error will be accepted
		const float errorDeadband = 0.5;

		// Maximum current value, upper and lower bounds, adjusted by throttle
		const float maxCurrentUpper = 15;
		const float maxCurrentLower = 10;

		std::shared_ptr<Victor> motorControllers[NUM_MANIPULATOR_MOTORS];
		std::shared_ptr<AnalogPotentiometer> potentiometers[NUM_MANIPULATOR_MOTORS];

		float destPosition[NUM_MANIPULATOR_MOTORS];
		float trackPosition[NUM_MANIPULATOR_MOTORS];

		float lastSpeed[NUM_MANIPULATOR_MOTORS];
		float lastPower[NUM_MANIPULATOR_MOTORS];
		float integralAccumulator[NUM_MANIPULATOR_MOTORS];
		float capPower[NUM_MANIPULATOR_MOTORS+1];

		uint32_t lastRunTimestamp;
		Joystick *joystick;
		Safety *safety;
		Preferences *perfs;
};

#endif /* SRC_MANIPULATOR_H_ */
