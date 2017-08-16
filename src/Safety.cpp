#include <Safety.h>

Safety::Safety(Joystick *controller, PowerDistributionPanel *pdpanel)
{
	powerRelay = std::make_shared<DigitalOutput>(relayPin);
	powerRelay->Set(1);

	this->joystick = controller;
	this->pdp = pdpanel;
	reset();
}

void Safety::update()
{
	uint32_t timestampMicros = getTimestampMicros();

	if(timestampMicros - lastRunTimestamp < safetyPeriod)
		return;

	lastRunTimestamp = timestampMicros;

	// Get max current setting

	float maxCurrent = map(joystick->GetRawAxis(CurrentLimit), 1, -1, maxCurrentLower, maxCurrentUpper);
	float current = 0;

	// Test whether all motor currents are within limits

	for(unsigned i = 0; i < DriveMotors::NUM_DRIVE_MOTORS; ++i)
	{
		current = (float)(pdp->GetCurrent(drivePowerChannels[i]));
		lastDriveSafetyCurrent[i] = ((1-currentSafetyFilter) * lastDriveSafetyCurrent[i]) + currentSafetyFilter * current;
		lastDriveControlCurrent[i] = ((1-currentControlFilter) * lastDriveControlCurrent[i]) + currentControlFilter * current;

		if (lastDriveSafetyCurrent[i] > maxCurrent)
		{
			powerRelay->Set(0);
			return;
		}
	}
	for(unsigned i = 0; i < ManipulatorMotors::NUM_MANIPULATOR_MOTORS+1; ++i)
	{
		current = (float)(pdp->GetCurrent(manipulatorPowerChannels[i]));
		lastManipulatorSafetyCurrent[i] = ((1-currentSafetyFilter) * lastManipulatorSafetyCurrent[i]) + currentSafetyFilter * current;
		lastManipulatorControlCurrent[i] = ((1-currentControlFilter) * lastManipulatorControlCurrent[i]) + currentControlFilter * current;

		if (lastManipulatorSafetyCurrent[i] > maxCurrent)
		{
			powerRelay->Set(0);
			return;
		}
	}

	powerRelay->Set(1);
}

void Safety::reset()
{
	for(unsigned i = 0; i < DriveMotors::NUM_DRIVE_MOTORS; ++i)
	{
		lastDriveSafetyCurrent[i] = 0;
		lastDriveControlCurrent[i] = 0;
	}
	for(unsigned i = 0; i < ManipulatorMotors::NUM_MANIPULATOR_MOTORS+1; ++i)
	{
		lastManipulatorSafetyCurrent[i] = 0;
		lastManipulatorControlCurrent[i] = 0;
	}
	lastRunTimestamp = getTimestampMicros() - safetyPeriod;
}

