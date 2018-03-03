#include <Drive.h>

Drive::Drive(Joystick *controller, Safety *safe)
{
	for(unsigned i = 0; i < NUM_DRIVE_MOTORS; ++i)
	{
		motorControllers[i] = std::make_shared<Victor>(driveMotorPins[i]);
		encoders[i] = std::make_shared<Encoder>(driveEncoderPins[i][0], driveEncoderPins[i][1]);
	}

	this->joystick = controller;
	this->safety = safe;
	reset();
}

void Drive::update()
{
	uint32_t timestampMicros = getTimestampMicros();
	if(timestampMicros - lastRunTimestamp < drivePeriod) return;
	lastRunTimestamp = timestampMicros;

	// Get max current setting
	float maxCurrent = map(joystick->GetRawAxis(CurrentLimit), -1, 1, maxCurrentLower, maxCurrentUpper);

	// Get motor currents and use it to adjust the power cap
	float current = 0;
	for(unsigned i = 0; i < NUM_DRIVE_MOTORS; ++i)
	{
		current = safety->getDriveCurrent(i);
		if(current > maxCurrent)
			capPower[i] -= powerChangeMax;
		else if(current < (maxCurrent-1))
			capPower[i] += powerChangeMax/5;
		capPower[i] = constrain(capPower[i], 0, 1);
	}

	// Calculate desired motor speeds from joystick input
	// Rover will not drive (hold at zero speed) unless the DriveRun button on the joystick is held

	float forwardSpeed = joystick->GetRawAxis(DriveForward);
	float turnSpeed = joystick->GetRawAxis(DriveTurn);
	if(!joystick->GetRawButton(DriveRun)) forwardSpeed = turnSpeed = 0;

	float leftSpeed = constrain(forwardSpeed + turnSpeed, -1, 1);
	float rightSpeed = constrain(forwardSpeed - turnSpeed, -1, 1);
	leftSpeed = map(leftSpeed, -1, 1, -maxSpeed, maxSpeed);
	rightSpeed = map(rightSpeed, -1, 1, -maxSpeed, maxSpeed);

	// Calculate current motor speeds
	int motorSpeed[NUM_DRIVE_MOTORS], encoderVal = 0;

	for(unsigned i = 0; i < NUM_DRIVE_MOTORS; ++i)
	{
		encoderVal = encoders[i]->GetRaw();
		motorSpeed[i] = encoderVal - lastEncoder[i];
		lastEncoder[i] = encoderVal;
	}

	// Perform motor saturation compensation by changing the desired speeds if a motor is saturated
	// This helps maintain the drive trajectory even if a motor if facing increased torque loading
	// Can be overridden by holding the DriveOverride button on the joystick (helpful if rover is stuck)

	float adjLeftSpeed = leftSpeed, adjRightSpeed = rightSpeed;
	if(!joystick->GetRawButton(DriveOverride))
	{
		float ratio = 0;
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
		adjLeftSpeed = leftSpeed * ratio;
		adjRightSpeed = rightSpeed * ratio;
	}

	// Perform integral control to obtain desired motor speed
	// Motor power is set to zero if the DriveEnable button on the joystick is not held

	if(joystick->GetRawButton(DriveEnable))
	{
		float speedError = 0;
		for(unsigned i = 0; i < NUM_DRIVE_MOTORS; ++i)
		{
			if(i <= RightRearMotor)
				speedError = adjRightSpeed - motorSpeed[i];
			else
				speedError = adjLeftSpeed - motorSpeed[i];

			lastPower[i] += constrain(speedError * kIntegral, -powerChangeMax, powerChangeMax);
			lastPower[i] = constrain(lastPower[i], -(capPower[i] + 0.1), (capPower[i] + 0.1)); // Allow extra to see if motor is saturating
			motorControllers[i]->Set(constrain(lastPower[i], -capPower[i], capPower[i]));

		}
	}
	else
	{
		for(unsigned i = 0; i < NUM_DRIVE_MOTORS; ++i)
		{
			motorControllers[i]->Set(0);
			lastEncoder[i] = encoders[i]->GetRaw();
			lastPower[i] = 0;
			capPower[i] = 0;
		}
	}

	// Packet length = 3 * (10 + 6 * 4) = 102
	std::string data = "DRIVE:";
	data += numToString(joystick->GetRawButton(DriveEnable) ? 1 : 0);
	data += numToString(joystick->GetRawButton(DriveRun) ? 1 : 0);
	data += numToString(joystick->GetRawButton(DriveOverride) ? 1 : 0);
	data += numToString(maxCurrent/100.0);
	data += numToString(forwardSpeed);
	data += numToString(turnSpeed);
	data += numToString(leftSpeed/maxSpeed);
	data += numToString(rightSpeed/maxSpeed);
	data += numToString(adjLeftSpeed/maxSpeed);
	data += numToString(adjRightSpeed/maxSpeed);
	for(unsigned i = 0; i < NUM_DRIVE_MOTORS; ++i)
		data += numToString(motorSpeed[i]/maxSpeed);
	for(unsigned i = 0; i < NUM_DRIVE_MOTORS; ++i)
		data += numToString(lastPower[i]);
	for(unsigned i = 0; i < NUM_DRIVE_MOTORS; ++i)
		data += numToString(safety->getDriveCurrent(i)/100.0);
	for(unsigned i = 0; i < NUM_DRIVE_MOTORS; ++i)
		data += numToString(capPower[i]);
	data+=":DRIVE";
	std::cout << data << std::endl;

	/*
	SmartDashboard::PutNumber("Forward Speed Input", 0.9*forwardSpeed);
	SmartDashboard::PutNumber("Turn Speed Input", 0.9*turnSpeed);
	SmartDashboard::PutNumber("Left Desired Speed", 0.9*leftSpeed/maxSpeed);
	SmartDashboard::PutNumber("Right Desired Speed", 0.9*rightSpeed/maxSpeed);
	SmartDashboard::PutNumber("Left Adjusted Speed", 0.9*adjLeftSpeed/maxSpeed);
	SmartDashboard::PutNumber("Right Adjusted Speed", 0.9*adjRightSpeed/maxSpeed);

	for(unsigned i = 0; i < NUM_DRIVE_MOTORS; ++i) {
		SmartDashboard::PutNumber(driveMotorNames[i]+" Speed", 0.9*motorSpeed[i]/maxSpeed);
		SmartDashboard::PutNumber(driveMotorNames[i]+" Power", 0.9*lastPower[i]);
		SmartDashboard::PutNumber(driveMotorNames[i]+" Current", 0.9*safety->getDriveCurrent(i)/maxCurrent);
		SmartDashboard::PutNumber(driveMotorNames[i]+" Power Cap", 0.9*capPower[i]);
	}
	*/
}

void Drive::reset()
{
	for(unsigned i = 0; i < NUM_DRIVE_MOTORS; ++i)
	{
		motorControllers[i]->Set(0);
		lastEncoder[i] = encoders[i]->GetRaw();
		lastPower[i] = 0;
		capPower[i] = 0;
	}
	lastRunTimestamp = getTimestampMicros() - drivePeriod;
}
