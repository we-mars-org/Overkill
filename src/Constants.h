/*
 * DozersPins.h
 *
 *  Created on: Aug 18, 2016
 *      Author: Agoston
 */

#ifndef SRC_CONSTANTS_H_
#define SRC_CONSTANTS_H_

#include <cstdint>
#include <memory>
#include <cmath>

#include <WPILib.h>

enum JoystickAxes
{
	DriveForward = 1,
	DriveTurn = 4,
};

enum JoystickButtons
{
	DriveFullSpeed = 5,
	DriveOverride = 6,
};

enum DriveMotors
{
	RightFrontMotor = 0,
	RightMiddleMotor,
	RightRearMotor,
	LeftFrontMotor,
	LeftMiddleMotor,
	LeftRearMotor,
	NUM_DRIVE_MOTORS,
};

enum ManipulatorMotors
{
	BaseMotor = 0,
	ShoulderMotor,
	ElbowMotor,
	PitchMotor,
	YawMotor,
	RollMotor,
	GripperMotor,
	NUM_MANIPULATOR_MOTORS,
};

const uint8_t driveMotorPins[DriveMotors::NUM_DRIVE_MOTORS] =
{
	5,
	4,
	3,
	2,
	1,
	0
};

const uint8_t driveEncoderPins[DriveMotors::NUM_DRIVE_MOTORS][2] =
{
	{0, 1},
	{2, 3},
	{4, 5},
	{6, 7},
	{8, 9},
	{10, 11}
};

const uint8_t drivePowerChannels[DriveMotors::NUM_DRIVE_MOTORS] =
{
	0,
	1,
	2,
	15,
	14,
	13
};

const uint8_t manipulatorMotorPins[ManipulatorMotors::NUM_MANIPULATOR_MOTORS] =
{
	6,
	7,
	8,
	9,
	12,
	13,
	14
};

const uint8_t manipulatorPotentiometerPins[ManipulatorMotors::NUM_MANIPULATOR_MOTORS] =
{
	0,
	1,
	2,
	3,
	4,
	5,
	6
};

/**
 * Drive loop run period in microseconds
 */
const uint32_t drivePeriod = 50 * 1000;

/**
 * Manipulator loop run period in microseconds
 */
const uint32_t manipulatorPeriod = 25 * 1000;

/**
 * Safety loop run period in microseconds
 */
const uint32_t safetyPeriod = 100 * 1000;

/**
 * Gets FPGA Timestamp in microseconds. Rolls over in 71 minutes.
 * @return FPGA Timestamp in microseconds
 */
inline uint32_t getTimestampMicros()
{
	return (uint32_t) (Timer::GetFPGATimestamp() * (double) 1000000);
}

/**
 * Constrains a value to be within a specified range.
 * @param n value to constrain
 * @param lower lower limit
 * @param upper upper limit
 * @return constrained value
 */
inline float constrain(float n, float lower, float upper)
{
	if (n > upper) n = upper;
	if (n < lower) n = lower;
	return n;
}

/**
 * Linear transformation of a value.
 * @param x value to transform
 * @param in_min minimum value of input value
 * @param in_max maximum value of input value
 * @param out_min minimum value of output value
 * @param out_max maximum value of output value
 * @return value transformed to parameters passed in
 */
inline float map(float x, float in_min, float in_max, float out_min, float out_max)
{
	return ((((x - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min);
}

#endif /* SRC_CONSTANTS_H_ */
