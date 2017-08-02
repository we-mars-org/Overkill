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

	// Test whether all motor currents are within limits

	for(unsigned i = 0; i < DriveMotors::NUM_DRIVE_MOTORS; ++i)
	{
		lastDriveCurrent[i] = (0.5 * lastDriveCurrent[i]) + 0.5 * pdp->GetCurrent(drivePowerChannels[i]);
		if (lastDriveCurrent[i] > maxCurrent)
		{
			powerRelay->Set(0);
			return;
		}
	}
	for(unsigned i = 0; i < ManipulatorMotors::NUM_MANIPULATOR_MOTORS+1; ++i)
	{
		lastManipulatorCurrent[i] = ((1-currentFilter) * lastManipulatorCurrent[i]) + currentFilter * pdp->GetCurrent(manipulatorPowerChannels[i]);
		if (lastManipulatorCurrent[i] > maxCurrent)
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
		lastDriveCurrent[i] = 0;
	for(unsigned i = 0; i < ManipulatorMotors::NUM_MANIPULATOR_MOTORS+1; ++i)
		lastManipulatorCurrent[i] = 0;
	lastRunTimestamp = getTimestampMicros() - safetyPeriod;
}

