/*
 * DozerDrive.cpp
 *
 *  Created on: Aug 4, 2016
 *      Author: Agoston
 */

#include <Drive.h>

#include <iostream>

Drive::Drive(Joystick *joystick)
{
	for(unsigned i = 0; i < DriveMotors::NUM_DRIVE_MOTORS; ++i)
	{
		motorControllers[i] = std::make_shared<Victor>(driveMotorPins[i]);
		encoders[i] = std::make_shared<Encoder>(driveEncoderPins[i][0], driveEncoderPins[i][1]);
	}

	lastRunTimestamp = getTimestampMicros() - drivePeriod;

	for(unsigned i = 0; i < DriveMotors::NUM_DRIVE_MOTORS; ++i)
	{
		lastPowerVals[i] = 0;
		lastEncoderVals[i] = 0;
	}

	this->joystick = joystick;
}

Drive::~Drive()
{
	// TODO Auto-generated destructor stub
}

void Drive::update()
{
	uint32_t timestampMicros = getTimestampMicros();

	if(timestampMicros - lastRunTimestamp < drivePeriod)
		return;

	lastRunTimestamp = timestampMicros;

	float forwardSpeed = -joystick->GetRawAxis(JoystickAxes::DriveForward);
	float turnSpeed = joystick->GetRawAxis(JoystickAxes::DriveTurn);

	if(fabs(forwardSpeed) < 0.05)
		forwardSpeed = 0;

	if(fabs(turnSpeed) < 0.05)
		turnSpeed = 0;

	float leftSpeed = constrain(forwardSpeed + turnSpeed, -1, 1);
	float rightSpeed = -constrain(forwardSpeed - turnSpeed, -1, 1);

	bool halfSpeed = joystick->GetRawButton(JoystickButtons::DriveHalfSpeed);

	if(halfSpeed)
	{
		leftSpeed = map(leftSpeed, -1, 1, -maxSpeed / 2, maxSpeed / 2);
		rightSpeed = map(rightSpeed, -1, 1, -maxSpeed / 2, maxSpeed / 2);
	}
	else
	{
		leftSpeed = map(leftSpeed, -1, 1, -maxSpeed, maxSpeed);
		rightSpeed = map(rightSpeed, -1, 1, -maxSpeed, maxSpeed);
	}

	int encoderVals[DriveMotors::NUM_DRIVE_MOTORS];
	int motorSpeeds[DriveMotors::NUM_DRIVE_MOTORS];

	for(unsigned i = 0; i < DriveMotors::NUM_DRIVE_MOTORS; ++i)
	{
		encoderVals[i] = encoders[i]->GetRaw();
	}

	float speedErrorValues[DriveMotors::NUM_DRIVE_MOTORS];

	for(unsigned i = 0; i < DriveMotors::NUM_DRIVE_MOTORS; ++i)
	{
		motorSpeeds[i] = encoderVals[i] - lastEncoderVals[i];
		lastEncoderVals[i] = encoderVals[i];
	}

	if(!joystick->GetRawButton(JoystickButtons::DriveOverride))
	{
		// Perform motor saturation compensation by changing the desired speeds if a motor is saturated
		float adjLeftSpeed = leftSpeed, adjRightSpeed = rightSpeed;
		for(unsigned i = 0; i < DriveMotors::NUM_DRIVE_MOTORS; ++i)
		{
			if(i <= DriveMotors::RightRearMotor)
			{
				if(((adjRightSpeed > motorSpeeds[i]) && (lastPowerVals[i] > 1)) ||
						((adjRightSpeed < motorSpeeds[i]) && (lastPowerVals[i] < -1)))
					adjRightSpeed = motorSpeeds[i];
			}
			else
			{
				if(((adjLeftSpeed > motorSpeeds[i]) && (lastPowerVals[i] > 1)) ||
						((adjLeftSpeed < motorSpeeds[i]) && (lastPowerVals[i] < -1)))
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


	for(unsigned i = 0; i < DriveMotors::NUM_DRIVE_MOTORS; ++i)
	{
		if(i <= DriveMotors::RightRearMotor)
		{
			speedErrorValues[i] = rightSpeed - motorSpeeds[i];
		}
		else
		{
			speedErrorValues[i] = leftSpeed - motorSpeeds[i];
		}
		lastPowerVals[i] += speedErrorValues[i] * kIntegral;
		lastPowerVals[i] = constrain(lastPowerVals[i], -1.1, 1.1); // Allow above 1 to see if motor is saturating
		motorControllers[i]->Set(constrain(lastPowerVals[i], -1, 1));
	}
}

void Drive::reset()
{
	for(unsigned i = 0; i < DriveMotors::NUM_DRIVE_MOTORS; ++i)
	{
		lastPowerVals[i] = 0;
	}
}
