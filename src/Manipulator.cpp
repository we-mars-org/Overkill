#include <Manipulator.h>

Manipulator::Manipulator(Joystick *controller, Safety *safe)
{
	for(unsigned i = 0; i < NUM_MANIPULATOR_JOINTS; ++i)
	{
		motorControllers[i] = std::make_shared<Victor>(manipulatorMotorPins[i]);
		potentiometers[i] = std::make_shared<AnalogPotentiometer>(manipulatorPotentiometerPins[i],
				manipulatorPotentiometerScale[i], -manipulatorPotentiometerScale[i]*manipulatorPotentiometerOffset[i]);
	}

	this->joystick = controller;
	this->safety = safe;
	reset();
}

void Manipulator::update()
{
	uint32_t timestampMicros = getTimestampMicros();
	if(timestampMicros - lastRunTimestamp < manipulatorPeriod) return;
	lastRunTimestamp = timestampMicros;

	// Get max current setting
	float maxCurrent = map(joystick->GetRawAxis(CurrentLimit), -1, 1, maxCurrentLower, maxCurrentUpper);

	// Get motor currents and use it to adjust the power cap
	for(unsigned i = 0; i < NUM_MANIPULATOR_JOINTS; ++i)
	{
		float current = safety->getManipulatorCurrent(i);
		if(current > maxCurrent)
			capPower[i] -= 5*powerChangeMax;
		else if(current < (maxCurrent-1))
			capPower[i] += powerChangeMax/2;
		capPower[i] = constrain(capPower[i], 0, maxPower[i]);
	}

	// Get the target and current joint positions
    // The desired joint positions are read only if ManipulatorControllable button on the joystick is held

	float jointPosition[NUM_MANIPULATOR_JOINTS];

	for(unsigned i = 0; i < NUM_MANIPULATOR_JOINTS; ++i)
	{
		if(joystick->GetRawButton(ManipulatorControllable))
			destPosition[i] = map(joystick->GetRawAxis(ElevatorPosition+i), -1, 1, manipulatorJointLimits[i][0], manipulatorJointLimits[i][1]);

		destPosition[i] = constrain(destPosition[i], manipulatorJointLimits[i][0], manipulatorJointLimits[i][1]);
		jointPosition[i] = potentiometers[i]->Get();
	}

	// Calculate the trajectory from current position to the target position
	// The trajectory updates only if ManipulatorRun button on the joystick is held

	if(joystick->GetRawButton(ManipulatorRun))
	{
		float travelAngle = 0, capSpeed = 0;
		for(unsigned i = 0; i < NUM_MANIPULATOR_JOINTS; ++i)
		{
			travelAngle = destPosition[i] - trackPosition[i];
			if(travelAngle > 0.1) // Need to move in forward direction, positive speed
			{
				capSpeed = std::sqrt(2*maxAccel*travelAngle); // This allows us to slow down as we approach target
				capSpeed = std::min(capSpeed, maxSpeed);
				if(capSpeed > lastSpeed[i]) // If need to go faster or change direction, speed up by maxAccel until capSpeed
					lastSpeed[i] = std::min(lastSpeed[i] + maxAccel, capSpeed);
				else // If need to go slower, clamp down to capSpeed
					lastSpeed[i] = capSpeed;

			}
			else if(travelAngle < -0.1) // Need to move in reverse direction, negative speed
			{
				travelAngle = -travelAngle;
				capSpeed = std::sqrt(2*maxAccel*travelAngle); // This allows us to slow down as we approach target
				capSpeed = -std::min(capSpeed, maxSpeed);
				if(capSpeed < lastSpeed[i]) // If need to go faster or change direction, speed up by maxAccel until capSpeed
					lastSpeed[i] = std::max(lastSpeed[i] - maxAccel, capSpeed);
				else // If need to go slower, clamp down to capSpeed
					lastSpeed[i] = capSpeed;
			}
			else
				lastSpeed[i] = 0;
			lastSpeed[i] = constrain(lastSpeed[i], -maxSpeed, maxSpeed);
			trackPosition[i] += lastSpeed[i];
		}
	}
	else
	{
		for(unsigned i = 0; i < NUM_MANIPULATOR_JOINTS; ++i)
			lastSpeed[i] = 0;
	}

	// Perform proportional-integral control to obtain desired motor position
	// Motor power is set to zero if the ManipulatorEnable button on the joystick is not held

	if(joystick->GetRawButton(ManipulatorEnable))
	{
		float positionError = 0, powerChange = 0;
		for(unsigned i = 0; i < NUM_MANIPULATOR_JOINTS; ++i)
		{
			trackPosition[i] = constrain(trackPosition[i], manipulatorJointLimits[i][0], manipulatorJointLimits[i][1]);
			positionError = trackPosition[i] - jointPosition[i];

			powerChange = positionError * kProportional[i] + (positionError - lastError[i]) * kDerivative[i] - lastPower[i];
			powerChange = constrain(powerChange, -powerChangeMax, powerChangeMax);

			lastError[i] = positionError;
			lastPower[i] += powerChange;
			lastPower[i] = constrain(lastPower[i], -(capPower[i] + 0.1), (capPower[i] + 0.1)); // Allow extra to see if motor is saturating
			motorControllers[i]->Set(constrain(lastPower[i], -capPower[i], capPower[i]));
		}
	}
	else
	{
		for(unsigned i = 0; i < NUM_MANIPULATOR_JOINTS; ++i)
		{
			motorControllers[i]->Set(0);
			trackPosition[i] = potentiometers[i]->Get();
			destPosition[i] = potentiometers[i]->Get();
			lastSpeed[i] = 0;
			lastPower[i] = 0;
			lastError[i] = 0;
			capPower[i] = 0;
		}
	}

	// Packet length = 3 * (4 + 5 * 7) = 117
	std::string data = "MANIP:";
	data += numToString(joystick->GetRawButton(ManipulatorEnable) ? 1 : 0);
	data += numToString(joystick->GetRawButton(ManipulatorRun) ? 1 : 0);
	data += numToString(joystick->GetRawButton(ManipulatorControllable) ? 1 : 0);
	data += numToString(maxCurrent/100.0);
	for(unsigned i = 0; i < NUM_MANIPULATOR_JOINTS; ++i)
		data += numToString(map(destPosition[i],manipulatorJointLimits[i][0],manipulatorJointLimits[i][1],-0.99,0.99));
	for(unsigned i = 0; i < NUM_MANIPULATOR_JOINTS; ++i)
		data += numToString(map(trackPosition[i],manipulatorJointLimits[i][0],manipulatorJointLimits[i][1],-0.99,0.99));
	for(unsigned i = 0; i < NUM_MANIPULATOR_JOINTS; ++i)
		data += numToString(map(jointPosition[i],manipulatorJointLimits[i][0],manipulatorJointLimits[i][1],-0.99,0.99));
	for(unsigned i = 0; i < NUM_MANIPULATOR_JOINTS; ++i)
		data += numToString(lastSpeed[i]/maxSpeed);
	for(unsigned i = 0; i < NUM_MANIPULATOR_JOINTS; ++i)
		data += numToString(lastPower[i]);
	for(unsigned i = 0; i < NUM_MANIPULATOR_JOINTS; ++i)
		data += numToString(safety->getManipulatorCurrent(i)/100.0);
	for(unsigned i = 0; i < NUM_MANIPULATOR_JOINTS; ++i)
		data += numToString(capPower[i]);
	data += ":MANIP";
	std::cout << data << std::endl;

	/*
	for(unsigned i = 0; i < NUM_MANIPULATOR_JOINTS; ++i)
		std::cout << "Pot " << i << ": " << jointPosition[i] << std::endl;
	*/
}

void Manipulator::reset()
{
	for(unsigned i = 0; i < NUM_MANIPULATOR_JOINTS; ++i)
	{
		motorControllers[i]->Set(0);
		trackPosition[i] = potentiometers[i]->Get();
		destPosition[i] = potentiometers[i]->Get();
		lastSpeed[i] = 0;
		lastPower[i] = 0;
		lastError[i] = 0;
		capPower[i] = 0;
	}
	lastRunTimestamp = getTimestampMicros() - manipulatorPeriod;
}
