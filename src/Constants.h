#ifndef SRC_CONSTANTS_H_
#define SRC_CONSTANTS_H_

#include <math.h>
#include <string>
#include <iostream>

#include <WPILib.h>

// Safety loop run period in microseconds (unit is microseconds per cycle)
const uint32_t safetyPeriod = 100 * 1000;

// Drive loop run period in microseconds (unit is microseconds per cycle)
const uint32_t drivePeriod = 50 * 1000;

// Manipulator loop run period in microseconds (unit is microseconds per cycle)
const uint32_t manipulatorPeriod = 25 * 1000;

// Relay DIO pin number: high keeps relay on, low turns it off
const uint8_t relayPin = 25;

// Assign names to Joystick axes
enum JoystickAxes
{
	// On both joysticks
	CurrentLimit = 0,
	// On drive joystick (id: 0)
	DriveForward = 1,
	DriveTurn = 2,
	 // On manipulator joystick (id: 1)
	BasePosition = 1,
	ShoulderPosition = 2,
	ElbowPosition = 3,
	PitchPosition = 4,
	YawPosition = 5,
	RollPosition = 6,
	GripperPosition = 7
};

// Assign names to Joystick buttons
enum JoystickButtons
{
	// On drive joystick (id: 0)
	DriveEnable = 1,
	DriveRun = 2,
	DriveOverride = 3,
	// On manipulator joystick (id: 1)
	ManipulatorEnable = 1,
	ManipulatorRun = 2,
	ManipulatorControllable = 3
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
	NUM_DRIVE_MOTORS
};

// Array associating motor names to corresponding Drive Motor IDs (array index)
const std::string driveMotorNames[NUM_DRIVE_MOTORS] =
{
	"Right Front",
	"Right Middle",
	"Right Rear",
	"Left Front",
	"Left Middle",
	"Left Rear"
};

// Array associating PWM pins to corresponding Drive Motor IDs (array index)
const uint8_t driveMotorPins[NUM_DRIVE_MOTORS] =
{
	3,
	4,
	5,
	0,
	1,
	2
};

// Array associating PDP current measure channels to corresponding Drive Motor IDs (array index)
const uint8_t drivePowerChannels[NUM_DRIVE_MOTORS] =
{
	9,
	10,
	11,
	6,
	5,
	4
};

// Array associating encoder DIO pins to corresponding Drive Motor IDs (array index)
const uint8_t driveEncoderPins[NUM_DRIVE_MOTORS][2] =
{
	{7, 6},
	{9, 8},
	{15, 14},
	{0, 1},
	{2, 3},
	{4, 5}
};

// Assign IDs to Manipulator Motors for use with other const arrays defined below
enum ManipulatorMotors
{
	BaseMotor = 0,
	ShoulderMotor1,
	ElbowMotor,
	PitchMotor,
	YawMotor,
	RollMotor,
	GripperMotor,
	ShoulderMotor2,
	NUM_MANIPULATOR_MOTORS
};

// Assign IDs to Manipulator Joints for use with other const arrays defined below
enum ManipulatorJoints
{
	BaseJoint = 0,
	ShoulderJoint,
	ElbowJoint,
	PitchJoint,
	YawJoint,
	RollJoint,
	GripperJoint,
	NUM_MANIPULATOR_JOINTS
};

// Array associating motor names to corresponding Manipulator Motor IDs (array index)
const std::string manipulatorMotorNames[NUM_MANIPULATOR_MOTORS] =
{
	"Base",
	"Shoulder 1",
	"Elbow",
	"Pitch",
	"Yaw",
	"Roll",
	"Gripper",
	"Shoulder 2"
};

// Array associating joint names to corresponding Manipulator Motor IDs (array index)
const std::string manipulatorJointNames[NUM_MANIPULATOR_JOINTS] =
{
	"Base",
	"Shoulder",
	"Elbow",
	"Pitch",
	"Yaw",
	"Roll",
	"Gripper"
};

// Array associating PWM pins to corresponding Manipulator Motor IDs (array index)
const uint8_t manipulatorMotorPins[NUM_MANIPULATOR_JOINTS] =
{
	17,
	16,
	15,
	14,
	13,
	12,
	11
};

// Array associating PDP current measure channels to corresponding Manipulator Motor IDs (array index)
const uint8_t manipulatorPowerChannels[NUM_MANIPULATOR_MOTORS] =
{
	0,
	1,
	3,
	15,
	14,
	13,
	12,
	2
};

// Array associating potentiometer AI pins to corresponding Manipulator Motor IDs (array index)
const uint8_t manipulatorPotentiometerPins[NUM_MANIPULATOR_JOINTS] =
{
	0,
	1,
	2,
	3,
	7,
	6,
	5
};

// Array associating potentiometer scaling value (covert to degrees) to corresponding Manipulator Motor IDs (array index)
const float manipulatorPotentiometerScale[NUM_MANIPULATOR_JOINTS] =
{
	-240,
	 240,
	-240,
	 240,
	-240,
	 240,
	-240
};

// Array associating potentiometer offset value (reading when joint is at 0 degrees) to corresponding Manipulator Motor IDs (array index)
const float manipulatorPotentiometerOffset[NUM_MANIPULATOR_JOINTS] =
{
	0.540,
	0.325,
	0.493,
	0.470,
	0.485,
	0.460,
	0.625
};

// Array associating joint limits (in degrees) to corresponding Manipulator Motor IDs (array index)
const float manipulatorJointLimits[NUM_MANIPULATOR_JOINTS][2] =
{
	{-60, 60},   // Hard stop at 65
	{-40, 90},   // Hard stop at 45 & 95
	{-110, 110}, // Hard stop at 147
	{-105, 105}, // Hard stop at 146
	{-110, 110}, // Hard stop at 140
	{-105, 105}, // Hard stop at 146
	{-50, 50}    // Hard stop at 54
};

/**
 * Gets FPGA Timestamp in microseconds. Rolls over in 71 minutes.
 * @return FPGA Timestamp in microseconds
 */
inline uint32_t getTimestampMicros()
{
	return (uint32_t) (Timer::GetFPGATimestamp() * 1000000);
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
	if (n > upper) return upper;
	if (n < lower) return lower;
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

/**
 * Convert a floating point number in range -1.0 > 0.0 > 1.0 to a string in range "-99" > "+00" > "+99"
 * @param x value to convert (range -1.0 to 1.0)
 * @return three character (sign + 2 digits) string representation of x
 */
inline std::string numToString(float x)
{
	int n = (int)(x * 100 + 0.5);
	if (n > 99) n = 99;
	if (n < -99) n = -99;
	std::string sign = "+";
	if(n < 0) { sign = "-"; n = -n; }
	return sign + (char)((int)(n/10)+'0') + (char)((int)(n%10)+'0');
}

#endif /* SRC_CONSTANTS_H_ */
