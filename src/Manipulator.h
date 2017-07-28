#ifndef SRC_MANIPULATOR_H_
#define SRC_MANIPULATOR_H_

#include <Constants.h>

class Manipulator
{
	public:
		Manipulator(Joystick *controller, PowerDistributionPanel *pdp);
		void update();
		void reset();

	protected:

	private:
		// Maximum joint angle velocity (in degrees per second)
		const double maxSpeed = 100.0;

		// Maximum joint angle acceleration (in degrees per second squared)
		const double maxAccel = 100.0;

		std::shared_ptr<Victor> motorControllers[ManipulatorMotors::NUM_MANIPULATOR_MOTORS];
		std::shared_ptr<AnalogPotentiometer> potentiometers[ManipulatorMotors::NUM_MANIPULATOR_MOTORS];

		double setPosition[ManipulatorMotors::NUM_MANIPULATOR_MOTORS];
		double destPosition[ManipulatorMotors::NUM_MANIPULATOR_MOTORS];
		double lastPosition[ManipulatorMotors::NUM_MANIPULATOR_MOTORS];

		uint32_t lastRunTimestamp;
		Joystick *joystick;
		PowerDistributionPanel *pdp;
};

#endif /* SRC_MANIPULATOR_H_ */
