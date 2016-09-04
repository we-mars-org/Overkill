/*
 * Safety.cpp
 *
 *  Created on: Sep 3, 2016
 *      Author: Agoston
 */

#include <Safety.h>

Safety::Safety(Joystick *joystick)
{
	powerRelay[0] = std::make_shared<DigitalOutput>(24);
	powerRelay[1] = std::make_shared<DigitalOutput>(25);
	
	powerRelay[0]->Set(0);
	powerRelay[1]->Set(1);

	lastRunTimestamp = getTimestampMicros() - safetyPeriod;
	this->joystick = joystick;
}

Safety::~Safety()
{
	// TODO Auto-generated destructor stub
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
