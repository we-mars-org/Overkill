#include <Safety.h>

Safety::Safety(Joystick *joystick, PowerDistributionPanel *pdp)
{
	powerRelay[0] = std::make_shared<DigitalOutput>(24);
	powerRelay[1] = std::make_shared<DigitalOutput>(25);
	
	powerRelay[0]->Set(0);
	powerRelay[1]->Set(1);

	this->joystick = joystick;
	this->pdp = pdp;
	reset();
}

void Safety::update()
{
	uint32_t timestampMicros = getTimestampMicros();

	if(timestampMicros - lastRunTimestamp < safetyPeriod)
		return;

	lastRunTimestamp = timestampMicros;

	powerRelay[0]->Set(0);
	powerRelay[1]->Set(1);
}

void Safety::reset()
{
	lastRunTimestamp = getTimestampMicros() - safetyPeriod;
}

