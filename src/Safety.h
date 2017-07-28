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
		// Maximum current that any motor is allowed to reach to. If the current goes higher, the rover shuts down.
		const float maxCurrent = 20;

		// Current LPF parameter; 1 = fastest response, 0 = no update
		const float currentFilter = 0.4;

		float lastDriveCurrentVals[DriveMotors::NUM_DRIVE_MOTORS];
		float lastManipulatorCurrentVals[ManipulatorMotors::NUM_MANIPULATOR_MOTORS+1];

		std::shared_ptr<DigitalOutput> powerRelay;
		uint32_t lastRunTimestamp;
		Joystick *joystick;
		PowerDistributionPanel *pdp;
};

#endif /* SRC_SAFETY_H_ */
