#include <Constants.h>
#include <Safety.h>
#include <Drive.h>
#include <Manipulator.h>

//FRC Code 2018: M83X18842

class Robot: public SampleRobot
{
private:
	Joystick joystickDrive;
	Joystick joystickManipulator;
	PowerDistributionPanel pdp;
	Safety safety;
	Drive drive;
	Manipulator manipulator;

public:
	Robot() :
			joystickDrive(0),
			joystickManipulator(1),
			pdp(),
			safety(&joystickDrive, &joystickManipulator, &pdp),
			drive(&joystickDrive, &safety),
			manipulator(&joystickManipulator, &safety)
	{
	}

	void RobotInit()
	{
		safety.reset();
		drive.reset();
		manipulator.reset();
	}

	void Disabled()
	{
		while (IsDisabled())
		{
			safety.update();
			drive.reset();
			manipulator.reset();
		}
	}

	void OperatorControl()
	{
		safety.reset();
		drive.reset();
		manipulator.reset();
		while (IsOperatorControl() && IsEnabled())
		{
			safety.update();
			drive.update();
			manipulator.update();
		}
	}
};

START_ROBOT_CLASS(Robot)
