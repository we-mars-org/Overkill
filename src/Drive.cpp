#include <Drive.h>

Drive::Drive(Joystick *controller, PowerDistributionPanel *pdpanel)
{
	for(unsigned i = 0; i < NUM_DRIVE_MOTORS; ++i)
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

	float maxCurrent = map(joystick->GetRawAxis(CurrentLimit), 1, -1, maxCurrentLower, maxCurrentUpper);

	// Get motor currents and use it to adjust the power cap

	float current = 0;
	for(unsigned i = 0; i < NUM_DRIVE_MOTORS; ++i)
	{
		current = pdp->GetCurrent(drivePowerChannels[i]);
		lastCurrent[i] = ((1-currentFilter) * lastCurrent[i]) + currentFilter * current;
		if(lastCurrent[i] > maxCurrent)
			capPower[i] -= powerChangeMax;
		else if(lastCurrent[i] < (maxCurrent-1))
			capPower[i] += powerChangeMax/5;
		capPower[i] = constrain(capPower[i], 0, 1);

		SmartDashboard::PutNumber(driveMotorNames[i]+" Current", round(current, 0.1));
		SmartDashboard::PutNumber(driveMotorNames[i]+" Power Cap", round(capPower[i], 0.01));
	}

	// Calculate desired motor speeds from joystick input
	// The max speed is limited to 50% unless the DriveFullSpeed button on the joystick is held
	// Rover will not drive (hold at zero speed) unless the DriveEnable button on the joystick is held

	float forwardSpeed = -joystick->GetRawAxis(DriveForward);
	float turnSpeed = joystick->GetRawAxis(DriveTurn);

	if(!joystick->GetRawButton(DriveEnable))
		forwardSpeed = turnSpeed = 0;

	SmartDashboard::PutNumber("Forward Speed Input", round(forwardSpeed, 0.01));
	SmartDashboard::PutNumber("Turn Speed Input", round(turnSpeed, 0.01));

	if(fabs(forwardSpeed) < 0.05)
		forwardSpeed = 0;

	if(fabs(turnSpeed) < 0.05)
		turnSpeed = 0;

	float leftSpeed = constrain(forwardSpeed + turnSpeed, -1, 1);
	float rightSpeed = constrain(forwardSpeed - turnSpeed, -1, 1);

	if(joystick->GetRawButton(DriveFullSpeed))
	{
		leftSpeed = map(leftSpeed, -1, 1, -maxSpeed, maxSpeed);
		rightSpeed = map(rightSpeed, -1, 1, -maxSpeed, maxSpeed);
	}
	else // Half speed
	{
		leftSpeed = map(leftSpeed, -1, 1, -maxSpeed / 2, maxSpeed / 2);
		rightSpeed = map(rightSpeed, -1, 1, -maxSpeed / 2, maxSpeed / 2);
	}

	SmartDashboard::PutNumber("Left Desired Speed", round(leftSpeed,1));
	SmartDashboard::PutNumber("Right Desired Speed", round(rightSpeed,1));

	// Calculate current motor speeds

	int motorSpeed[NUM_DRIVE_MOTORS], encoderVal = 0;

	for(unsigned i = 0; i < NUM_DRIVE_MOTORS; ++i)
	{
		encoderVal = encoders[i]->GetRaw();
		motorSpeed[i] = encoderVal - lastEncoder[i];
		lastEncoder[i] = encoderVal;

		SmartDashboard::PutNumber(driveMotorNames[i]+" Speed", round(motorSpeed[i],1));
	}

	// Perform motor saturation compensation by changing the desired speeds if a motor is saturated
	// This helps maintain the drive trajectory even if a motor if facing increased torque loading
	// Can be overridden by holding the DriveOverride button on the joystick (helpful if rover is stuck)

	if(!joystick->GetRawButton(DriveOverride))
	{
		float adjLeftSpeed = leftSpeed, adjRightSpeed = rightSpeed, ratio = 0;
		for(unsigned i = 0; i < NUM_DRIVE_MOTORS; ++i)
		{
			if(i <= RightRearMotor)
			{
				if(((adjRightSpeed > motorSpeed[i]) && (lastPower[i] > capPower[i])) ||
						((adjRightSpeed < motorSpeed[i]) && (lastPower[i] < -capPower[i])))
					adjRightSpeed = motorSpeed[i];
			}
			else
			{
				if(((adjLeftSpeed > motorSpeed[i]) && (lastPower[i] > capPower[i])) ||
						((adjLeftSpeed < motorSpeed[i]) && (lastPower[i] < -capPower[i])))
					adjLeftSpeed = motorSpeed[i];
			}
		}
		adjLeftSpeed = (leftSpeed == 0) ? 1 : (adjLeftSpeed / leftSpeed);
		adjRightSpeed = (rightSpeed == 0) ? 1 : (adjRightSpeed / rightSpeed);
		ratio = std::min(adjLeftSpeed, adjRightSpeed);
		ratio = constrain(ratio, -1, 1);
		leftSpeed *= ratio;
		rightSpeed *= ratio;
	}

	SmartDashboard::PutNumber("Left Adjusted Speed", round(leftSpeed,1));
	SmartDashboard::PutNumber("Right Adjusted Speed", round(rightSpeed,1));

	// Perform integral control to obtain desired motor speed

	float speedError = 0, powerError = 0;

	for(unsigned i = 0; i < NUM_DRIVE_MOTORS; ++i)
	{
		if(i <= RightRearMotor)
			speedError = rightSpeed - motorSpeed[i];
		else
			speedError = leftSpeed - motorSpeed[i];

		powerError = speedError * kIntegral;

		if(powerError > powerChangeThresh)
			powerError = constrain(powerError, powerChangeMin, powerChangeMax);
		else if(powerError < -powerChangeThresh)
			powerError = constrain(powerError, -powerChangeMax, -powerChangeMin);
		else
			powerError = 0;

		lastPower[i] += powerError;
		lastPower[i] = constrain(lastPower[i], -(capPower[i] + 0.1), (capPower[i] + 0.1)); // Allow extra to see if motor is saturating
		motorControllers[i]->Set(constrain(lastPower[i], -capPower[i], capPower[i]));

		SmartDashboard::PutNumber(driveMotorNames[i]+" Power", round(lastPower[i],0.01));
	}
}

void Drive::reset()
{
	for(unsigned i = 0; i < NUM_DRIVE_MOTORS; ++i)
	{
		motorControllers[i]->Set(0);
		lastEncoder[i] = encoders[i]->GetRaw();
		lastPower[i] = 0;
		lastCurrent[i] = 0;
		capPower[i] = 0;
	}
	lastRunTimestamp = getTimestampMicros() - drivePeriod;
}
