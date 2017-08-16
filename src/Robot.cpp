#include <Constants.h>
#include <Safety.h>
#include <Drive.h>
#include <Manipulator.h>

//FRC Code: M82X12516

class Robot: public SampleRobot
{
private:
	Joystick joystick;
	PowerDistributionPanel pdp;
	Safety safety;
	Drive drive;
	Manipulator manipulator;

public:
	Robot() :
			joystick(0),
			pdp(),
			safety(&joystick, &pdp),
			drive(&joystick, &safety),
			manipulator(&joystick, &safety)
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
