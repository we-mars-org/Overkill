#include <Drive.h>

Drive::Drive(Joystick *controller, PowerDistributionPanel *pdpanel)
{
	for(unsigned i = 0; i < DriveMotors::NUM_DRIVE_MOTORS; ++i)
	{
		motorControllers[i] = std::make_shared<Victor>(driveMotorPins[i]);
		encoders[i] = std::make_shared<Encoder>(driveEncoderPins[i][0], driveEncoderPins[i][1]);
	}

	this->joystick = controller;
	this->pdp = pdpanel;
	reset();
}

void Drive::update()
{
	uint32_t timestampMicros = getTimestampMicros();

	if(timestampMicros - lastRunTimestamp < drivePeriod)
		return;

	lastRunTimestamp = timestampMicros;

	// Get max current setting

	float maxCurrent = map(joystick->GetRawAxis(JoystickAxes::CurrentLimit), 1, -1, maxCurrentLower, maxCurrentUpper);

	// Get motor currents and use it to adjust the power cap

	for(unsigned i = 0; i < DriveMotors::NUM_DRIVE_MOTORS; ++i)
	{
		lastCurrentVals[i] = ((1-currentFilter) * lastCurrentVals[i]) + currentFilter * pdp->GetCurrent(drivePowerChannels[i]);
		if(lastCurrentVals[i] > maxCurrent)
			capPowerVals[i] -= powerChangeMax;
		else
			capPowerVals[i] += powerChangeMax;
		capPowerVals[i] = constrain(capPowerVals[i], 0, 1);
	}

	// Calculate desired motor speeds from joystick input
	// The max speed is limited to 50% unless the DriveFullSpeed button on the joystick is held

	float forwardSpeed = -joystick->GetRawAxis(JoystickAxes::DriveForward);
	float turnSpeed = joystick->GetRawAxis(JoystickAxes::DriveTurn);

	if(fabs(forwardSpeed) < 0.05)
		forwardSpeed = 0;

	if(fabs(turnSpeed) < 0.05)
		turnSpeed = 0;

	float leftSpeed = constrain(forwardSpeed + turnSpeed, -1, 1);
	float rightSpeed = constrain(forwardSpeed - turnSpeed, -1, 1);

	if(joystick->GetRawButton(JoystickButtons::DriveFullSpeed))
	{
		leftSpeed = map(leftSpeed, -1, 1, -maxSpeed, maxSpeed);
		rightSpeed = map(rightSpeed, -1, 1, -maxSpeed, maxSpeed);
	}
	else // Half speed
	{
		leftSpeed = map(leftSpeed, -1, 1, -maxSpeed / 2, maxSpeed / 2);
		rightSpeed = map(rightSpeed, -1, 1, -maxSpeed / 2, maxSpeed / 2);
	}

	// Calculate current motor speeds

	int motorSpeeds[DriveMotors::NUM_DRIVE_MOTORS], encoderVal = 0;

	for(unsigned i = 0; i < DriveMotors::NUM_DRIVE_MOTORS; ++i)
	{
		encoderVal = encoders[i]->GetRaw();
		motorSpeeds[i] = encoderVal - lastEncoderVals[i];
		lastEncoderVals[i] = encoderVal;
	}

	// Perform motor saturation compensation by changing the desired speeds if a motor is saturated
	// This helps maintain the drive trajectory even if a motor if facing increased torque loading
	// Can be overridden by holding the DriveOverride button on the joystick (helpful if rover is stuck)

	if(!joystick->GetRawButton(JoystickButtons::DriveOverride))
	{
		float adjLeftSpeed = leftSpeed, adjRightSpeed = rightSpeed;
		for(unsigned i = 0; i < DriveMotors::NUM_DRIVE_MOTORS; ++i)
		{
			if(i <= DriveMotors::RightRearMotor)
			{
				if(((adjRightSpeed > motorSpeeds[i]) && (lastPowerVals[i] > capPowerVals[i])) ||
						((adjRightSpeed < motorSpeeds[i]) && (lastPowerVals[i] < -capPowerVals[i])))
					adjRightSpeed = motorSpeeds[i];
			}
			else
			{
				if(((adjLeftSpeed > motorSpeeds[i]) && (lastPowerVals[i] > capPowerVals[i])) ||
						((adjLeftSpeed < motorSpeeds[i]) && (lastPowerVals[i] < -capPowerVals[i])))
					adjLeftSpeed = motorSpeeds[i];
			}
		}
		adjLeftSpeed = (leftSpeed == 0) ? 1 : (adjLeftSpeed / leftSpeed);
		adjRightSpeed = (rightSpeed == 0) ? 1 : (adjRightSpeed / rightSpeed);
		float ratio = std::min(adjLeftSpeed, adjRightSpeed);
		ratio = constrain(ratio, -1, 1);
		leftSpeed *= ratio;
		rightSpeed *= ratio;
	}

	// Perform integral control to obtain desired motor speed

	float speedErrorValue = 0, powerErrorValue = 0;

	for(unsigned i = 0; i < DriveMotors::NUM_DRIVE_MOTORS; ++i)
	{
		if(i <= DriveMotors::RightRearMotor)
			speedErrorValue = rightSpeed - motorSpeeds[i];
		else
			speedErrorValue = leftSpeed - motorSpeeds[i];

		powerErrorValue = speedErrorValue * kIntegral;

		if(powerErrorValue > powerChangeThresh)
			powerErrorValue = constrain(powerErrorValue, powerChangeMin, powerChangeMax);
		else if(powerErrorValue < -powerChangeThresh)
			powerErrorValue = constrain(powerErrorValue, -powerChangeMax, -powerChangeMin);
		else
			powerErrorValue = 0;

		lastPowerVals[i] += powerErrorValue;
		lastPowerVals[i] = constrain(lastPowerVals[i], -(capPowerVals[i] + 0.1), (capPowerVals[i] + 0.1)); // Allow extra to see if motor is saturating
		motorControllers[i]->Set(constrain(lastPowerVals[i], -capPowerVals[i], capPowerVals[i]));

		std::cout << "Motor " << i << " Current = " << pdp->GetCurrent(drivePowerChannels[i]) << std::endl;
	}
}

void Drive::reset()
{
	for(unsigned i = 0; i < DriveMotors::NUM_DRIVE_MOTORS; ++i)
	{
		motorControllers[i]->Set(0);
		lastPowerVals[i] = 0;
		lastCurrentVals[i] = 0;
		capPowerVals[i] = 0;
		lastEncoderVals[i] = encoders[i]->GetRaw();
	}
	lastRunTimestamp = getTimestampMicros() - drivePeriod;
}
