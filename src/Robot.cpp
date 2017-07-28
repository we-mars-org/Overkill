#include <Constants.h>
#include <Drive.h>
#include <Manipulator.h>
#include <Safety.h>

//FRC Code: M82X12516

class Robot: public SampleRobot
{
	Joystick joystick;
	PowerDistributionPanel pdp;
	Drive drive;
	Manipulator manipulator;
	Safety safety;

public:
	Robot() :
			joystick(0),
			pdp(),
			drive(&joystick, &pdp),
			manipulator(&joystick, &pdp),
			safety(&joystick, &pdp)
	{
	}

	void RobotInit()
	{
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
