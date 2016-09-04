/*
 * Manipulator.cpp
 *
 *  Created on: Aug 11, 2016
 *      Author: Agoston
 */

#include <Manipulator.h>

Manipulator::Manipulator(Joystick *joystick)
{
	for(unsigned i = 0; i < ManipulatorMotors::NUM_MANIPULATOR_MOTORS; ++i)
	{
		motorControllers[i] = std::make_shared<Victor>(manipulatorMotorPins[i]);
		potentiometers[i] = std::make_shared<AnalogPotentiometer>(manipulatorPotentiometerPins[i]);
	}

	this->joystick = joystick;

	lastRunTimestamp = getTimestampMicros() - manipulatorPeriod;
}

Manipulator::~Manipulator()
{
	// TODO Auto-generated destructor stub
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
	// TODO Add reset code
}
