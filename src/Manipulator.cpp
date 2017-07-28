#include <Manipulator.h>

Manipulator::Manipulator(Joystick *controller, PowerDistributionPanel *pdpanel)
{
	for(unsigned i = 0; i < ManipulatorMotors::NUM_MANIPULATOR_MOTORS; ++i)
	{
		motorControllers[i] = std::make_shared<Victor>(manipulatorMotorPins[i]);
		potentiometers[i] = std::make_shared<AnalogPotentiometer>(manipulatorPotentiometerPins[i],
				manipulatorPotentiometerScale[i], manipulatorPotentiometerOffset[i]);
	}

	this->joystick = controller;
	this->pdp = pdpanel;
	reset();
}

void Manipulator::update()
{
	uint32_t timestampMicros = getTimestampMicros();

	if(timestampMicros - lastRunTimestamp < manipulatorPeriod)
		return;

	lastRunTimestamp = timestampMicros;

	//motorControllers[2]->Set(-joystick->GetRawAxis(JoystickAxes::DriveForward));
	//motorControllers[1]->Set(joystick->GetRawAxis(JoystickAxes::DriveTurn));

	//std::cout << "Motor Current = " << pdp->GetCurrent(manipulatorPowerChannels[1]) << std::endl;
	//std::cout << "Motor Current = " << pdp->GetCurrent(manipulatorPowerChannels[2]) << std::endl;
	//std::cout << "Motor Current = " << pdp->GetCurrent(manipulatorPowerChannels[7]) << std::endl;

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
