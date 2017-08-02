#ifndef SRC_SAFETY_H_
#define SRC_SAFETY_H_

#include <Constants.h>

class Safety
{
	public:
		Safety(Joystick *controller, PowerDistributionPanel *pdpanel);
		void update();
		void reset();

	private:
		// Maximum current that any motor is allowed to reach to, upper and lower bounds, adjusted by throttle
		// If the current goes higher than the max current here, the rover shuts down
		// Recommended to be at least 5 amps above the upper and lower bounds set in the drive and manipulator code
		const float maxCurrentUpper = 20;
		const float maxCurrentLower = 15;

		// Current measurement LPF parameter; 1 = fastest response, 0 = no response
		const float currentFilter = 0.4;

		float lastDriveCurrent[DriveMotors::NUM_DRIVE_MOTORS];
		float lastManipulatorCurrent[ManipulatorMotors::NUM_MANIPULATOR_MOTORS+1];

		std::shared_ptr<DigitalOutput> powerRelay;
		uint32_t lastRunTimestamp;
		Joystick *joystick;
		PowerDistributionPanel *pdp;
};

#endif /* SRC_SAFETY_H_ */
