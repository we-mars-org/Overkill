#ifndef SRC_SAFETY_H_
#define SRC_SAFETY_H_

#include <Constants.h>

class Safety
{
	public:
		Safety(Joystick *drive, Joystick *manipulator, PowerDistributionPanel *pdpanel);
		void update();
		void reset();
		float getDriveCurrent(unsigned ch) { return (ch < NUM_DRIVE_MOTORS) ? lastDriveControlCurrent[ch] : 0; }
		float getManipulatorCurrent(unsigned ch) { return (ch < NUM_MANIPULATOR_JOINTS) ? lastManipulatorControlCurrent[ch] : 0; }

	private:
		// Maximum current that any motor is allowed to reach to, upper and lower bounds, adjusted by throttle
		// If the current goes higher than the max current here, the rover shuts down
		// Recommended to be at least 5 amps above the upper and lower bounds set in the drive and manipulator code
		const float maxCurrentDriveUpper = 20;
		const float maxCurrentDriveLower = 15;
		const float maxCurrentManipulatorUpper = 20;
		const float maxCurrentManipulatorLower = 15;

		// Current measurement LPF parameter; 1 = fastest response, 0 = no response
		const float currentSafetyFilter = 0.2;
		const float currentControlFilter = 0.7;

		float lastDriveSafetyCurrent[NUM_DRIVE_MOTORS];
		float lastManipulatorSafetyCurrent[NUM_MANIPULATOR_JOINTS];
		float lastDriveControlCurrent[NUM_DRIVE_MOTORS];
		float lastManipulatorControlCurrent[NUM_MANIPULATOR_JOINTS];

		std::shared_ptr<DigitalOutput> powerRelay;
		uint32_t lastRunTimestamp;
		Joystick *joystickDrive;
		Joystick *joystickManipulator;
		PowerDistributionPanel *pdp;
};

#endif /* SRC_SAFETY_H_ */
