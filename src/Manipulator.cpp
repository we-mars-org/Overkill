#include <Manipulator.h>

Manipulator::Manipulator(Joystick *joystick, PowerDistributionPanel *pdp)
{
	for(unsigned i = 0; i < ManipulatorMotors::NUM_MANIPULATOR_MOTORS; ++i)
	{
		motorControllers[i] = std::make_shared<Victor>(manipulatorMotorPins[i]);
		potentiometers[i] = std::make_shared<AnalogPotentiometer>(manipulatorPotentiometerPins[i],
				manipulatorPotentiometerScale[i], manipulatorPotentiometerOffset[i]);
	}

	this->joystick = joystick;
	this->pdp = pdp;
	reset();
}

void Manipulator::update()
{
	uint32_t timestampMicros = getTimestampMicros();

	if(timestampMicros - lastRunTimestamp < manipulatorPeriod)
		return;

	lastRunTimestamp = timestampMicros;


}

void Manipulator::reset()
{
	for(unsigned i = 0; i < ManipulatorMotors::NUM_MANIPULATOR_MOTORS; ++i)
	{
		motorControllers[i]->Set(0);
		setPosition[i] = potentiometers[i]->Get();
		destPosition[i] = manipulatorPositionStow[i];
	}
	lastRunTimestamp = getTimestampMicros() - manipulatorPeriod;
}
