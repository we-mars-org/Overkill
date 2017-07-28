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

	for(unsigned i = 0; i < DriveMotors::NUM_DRIVE_MOTORS; ++i)
	{
		lastDriveCurrentVals[i] = (0.5 * lastDriveCurrentVals[i]) + 0.5 * pdp->GetCurrent(drivePowerChannels[i]);
		if (lastDriveCurrentVals[i] > maxCurrent)
		{
			powerRelay->Set(0);
			return;
		}
	}
	for(unsigned i = 0; i < ManipulatorMotors::NUM_MANIPULATOR_MOTORS+1; ++i)
	{
		lastManipulatorCurrentVals[i] = ((1-currentFilter) * lastManipulatorCurrentVals[i]) + currentFilter * pdp->GetCurrent(manipulatorPowerChannels[i]);
		if (lastManipulatorCurrentVals[i] > maxCurrent)
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
		lastDriveCurrentVals[i] = 0;
	for(unsigned i = 0; i < ManipulatorMotors::NUM_MANIPULATOR_MOTORS+1; ++i)
		lastManipulatorCurrentVals[i] = 0;
	lastRunTimestamp = getTimestampMicros() - safetyPeriod;
}

