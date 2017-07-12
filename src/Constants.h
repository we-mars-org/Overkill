#ifndef SRC_CONSTANTS_H_
#define SRC_CONSTANTS_H_

#include <cstdint>
#include <memory>
#include <cmath>

#include <WPILib.h>

// Assign names to Joystick axes
enum JoystickAxes
{
	DriveForward = 1,
	DriveTurn = 4,
};

// Assign names to Joystick buttons
enum JoystickButtons
{
	DriveFullSpeed = 5,
	DriveOverride = 6,
};

// Assign IDs to Drive Motors for use with other const arrays defined below
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

// Assign IDs to Manipulator Motors for use with other const arrays defined below
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

// Array associating PWM pins to corresponding Drive Motor IDs (array index)
const uint8_t driveMotorPins[DriveMotors::NUM_DRIVE_MOTORS] =
{
	5,
	4,
	3,
	2,
	1,
	0
};

// Array associating encoder DIO pins to corresponding Drive Motor IDs (array index)
const uint8_t driveEncoderPins[DriveMotors::NUM_DRIVE_MOTORS][2] =
{
	{0, 1},
	{2, 3},
	{4, 5},
	{6, 7},
	{8, 9},
	{10, 11}
};

// Array associating PDP current measure channels to corresponding Drive Motor IDs (array index)
const uint8_t drivePowerChannels[DriveMotors::NUM_DRIVE_MOTORS] =
{
	0,
	1,
	2,
	15,
	14,
	13
};

// Array associating PWM pins to corresponding Manipulator Motor IDs (array index)
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

// Array associating potentiometer AI pins to corresponding Manipulator Motor IDs (array index)
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

// Array associating potentiometer scaling value (covert to degrees) to corresponding Manipulator Motor IDs (array index)
const double manipulatorPotentiometerScale[ManipulatorMotors::NUM_MANIPULATOR_MOTORS] =
{
	1.0,
	1.0,
	1.0,
	1.0,
	1.0,
	1.0,
	1.0
};

// Array associating potentiometer offset value (in degrees) to corresponding Manipulator Motor IDs (array index)
const double manipulatorPotentiometerOffset[ManipulatorMotors::NUM_MANIPULATOR_MOTORS] =
{
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0
};

// Array associating joint limits (in degrees) to corresponding Manipulator Motor IDs (array index)
const double manipulatorJointLimits[ManipulatorMotors::NUM_MANIPULATOR_MOTORS][2] =
{
	{-65.0, 65.0}, // Hits hard stop at 70
	{-65.0, 65.0}, // Hits hard stop at 70
	{-140.0, 140.0}, // Hits hard stop at 147
	{-140.0, 140.0}, // Hits hard stop at 146
	{-135.0, 135.0}, // Hits hard stop at 140
	{-140.0, 140.0}, // Hits hard stop at 146
	{-50.0, 50.0} // Hits hard stop at 54
};

// Array associating stowed pose joint angles to corresponding Manipulator Motor IDs (array index)
const double manipulatorPositionStow[ManipulatorMotors::NUM_MANIPULATOR_MOTORS] =
{
	0.0,
	-65.0,
	140.0,
	0.0,
	-135.0,
	0.0,
	0.0
};

// Drive loop run period in microseconds
const uint32_t drivePeriod = 40 * 1000;

// Manipulator loop run period in microseconds
const uint32_t manipulatorPeriod = 20 * 1000;


// Safety loop run period in microseconds
const uint32_t safetyPeriod = 80 * 1000;

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
