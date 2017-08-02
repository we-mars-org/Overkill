#include <Manipulator.h>

Manipulator::Manipulator(Joystick *controller, PowerDistributionPanel *pdpanel)
{
	for(unsigned i = 0; i < NUM_MANIPULATOR_MOTORS; ++i)
	{
		motorControllers[i] = std::make_shared<Victor>(manipulatorMotorPins[i]);
		potentiometers[i] = std::make_shared<AnalogPotentiometer>(manipulatorPotentiometerPins[i],
				manipulatorPotentiometerScale[i], -manipulatorPotentiometerScale[i]*manipulatorPotentiometerOffset[i]);
	}

	this->joystick = controller;
	this->pdp = pdpanel;
	this->perfs = Preferences::GetInstance();
	reset();
}

void Manipulator::update()
{
	uint32_t timestampMicros = getTimestampMicros();

	if(timestampMicros - lastRunTimestamp < manipulatorPeriod)
		return;

	lastRunTimestamp = timestampMicros;

	// Get max current setting

	float maxCurrent = map(joystick->GetRawAxis(CurrentLimit), 1, -1, maxCurrentLower, maxCurrentUpper);

	// Get motor currents and use it to adjust the power cap

	for(unsigned i = 0; i < NUM_MANIPULATOR_MOTORS+1; ++i)
	{
		float current = pdp->GetCurrent(manipulatorPowerChannels[i]);
		lastCurrent[i] = ((1-currentFilter) * lastCurrent[i]) + currentFilter * current;
		if(lastCurrent[i] > maxCurrent)
			capPower[i] -= powerChangeMax;
		else if(lastCurrent[i] < (maxCurrent-1))
			capPower[i] += powerChangeMax/5;
		capPower[i] = constrain(capPower[i], 0, 1);

		SmartDashboard::PutNumber(manipulatorMotorNames[i]+" Current", round(current,0.1));
		SmartDashboard::PutNumber(manipulatorMotorNames[i]+" Power Cap", round(capPower[i],0.01));
	}
	capPower[ShoulderMotor2] = capPower[ShoulderMotor1] = std::min(capPower[ShoulderMotor1], capPower[ShoulderMotor2]);

	// Get the target and current joint positions
    // The desired joint positions are read from the SmartDashboard only if ManipulatorControllable button on the joystick is held

	float jointPosition[NUM_MANIPULATOR_MOTORS];

	for(unsigned i = 0; i < NUM_MANIPULATOR_MOTORS; ++i)
	{
		if(joystick->GetRawButton(ManipulatorControllable))
			destPosition[i] = perfs->GetDouble(manipulatorJointNames[i]+" Desired Angle", manipulatorPositionStow[i]);

		destPosition[i] = constrain(destPosition[i], manipulatorJointLimits[i][0], manipulatorJointLimits[i][1]);
		jointPosition[i] = potentiometers[i]->Get();

		SmartDashboard::PutNumber(manipulatorJointNames[i]+" Target Angle",  round(destPosition[i],0.1));
		SmartDashboard::PutNumber(manipulatorJointNames[i]+" Current Angle", round(jointPosition[i],0.1));
	}

	// Calculate the trajectory from current position to the target position
	// The trajectory updates only if ManipulatorEnable button on the joystick is held

	if(joystick->GetRawButton(ManipulatorEnable))
	{
		float travelAngle = 0, capSpeed = 0;
		for(unsigned i = 0; i < NUM_MANIPULATOR_MOTORS; ++i)
		{
			travelAngle = destPosition[i] - trackPosition[i];
			if(travelAngle > 0)
			{
				capSpeed = std::sqrt(2*maxAccel*travelAngle); // This allows us to slow down as we approach target
				capSpeed = std::min(capSpeed, maxSpeed);
				if(capSpeed > lastSpeed[i]) // If need to go faster or change direction, speed up by maxAccel until capSpeed
					lastSpeed[i] = std::min(lastSpeed[i] + maxAccel, capSpeed);
				else // If need to go slower, clamp down to capSpeed
					lastSpeed[i] = capSpeed;

			}
			else if(travelAngle < 0)
			{
				travelAngle = -travelAngle;
				capSpeed = std::sqrt(2*maxAccel*travelAngle); // This allows us to slow down as we approach target
				capSpeed = -std::min(capSpeed, maxSpeed);
				if(capSpeed < lastSpeed[i]) // If need to go faster or change direction, speed up by maxAccel until capSpeed
					lastSpeed[i] = std::max(lastSpeed[i] - maxAccel, capSpeed);
				else // If need to go slower, clamp down to capSpeed
					lastSpeed[i] = capSpeed;
			}
			trackPosition[i] += lastSpeed[i];
		}
	}
	else
	{
		for(unsigned i = 0; i < NUM_MANIPULATOR_MOTORS; ++i)
		{
			lastSpeed[i] = 0;
		}
	}

	// Perform proportional-integral control to obtain desired motor position

	float positionError = 0, powerError = 0;

	for(unsigned i = 0; i < NUM_MANIPULATOR_MOTORS; ++i)
	{
		trackPosition[i] = constrain(trackPosition[i], manipulatorJointLimits[i][0], manipulatorJointLimits[i][1]);
		positionError = trackPosition[i] - jointPosition[i];
		integralAccumulator[i] += positionError * kIntegral[i];
		integralAccumulator[i] = constrain(integralAccumulator[i], -kIntegralLimit, kIntegralLimit);
		powerError = (positionError * kProportional[i]) + integralAccumulator[i] - lastPower[i] ;

		if(powerError > powerChangeThresh)
			powerError = constrain(powerError, powerChangeMin, powerChangeMax);
		else if(powerError < -powerChangeThresh)
			powerError = constrain(powerError, -powerChangeMax, -powerChangeMin);
		else
			powerError = 0;

		lastPower[i] += powerError;
		lastPower[i] = constrain(lastPower[i], -(capPower[i] + 0.1), (capPower[i] + 0.1)); // Allow extra to see if motor is saturating
		motorControllers[i]->Set(constrain(lastPower[i], -capPower[i], capPower[i]));

		SmartDashboard::PutNumber(manipulatorJointNames[i]+" Track Angle",  round(trackPosition[i],0.1));
		SmartDashboard::PutNumber(manipulatorJointNames[i]+" Track Speed",  round(lastSpeed[i],0.01));
		SmartDashboard::PutNumber(manipulatorJointNames[i]+" Power", round(lastPower[i],0.01));
	}
}

void Manipulator::reset()
{
	for(unsigned i = 0; i < NUM_MANIPULATOR_MOTORS; ++i)
	{
		motorControllers[i]->Set(0);
		trackPosition[i] = potentiometers[i]->Get();
		destPosition[i] = manipulatorPositionStow[i];
		lastSpeed[i] = 0;
		lastPower[i] = 0;
		integralAccumulator[i] = 0;
		perfs->PutDouble(manipulatorJointNames[i]+" Desired Angle", manipulatorPositionStow[i]);
	}
	for(unsigned i = 0; i < NUM_MANIPULATOR_MOTORS+1; ++i)
	{
		lastCurrent[i] = 0;
		capPower[i] = 0;
	}
	lastRunTimestamp = getTimestampMicros() - manipulatorPeriod;
}
