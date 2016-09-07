/*
 * Manipulator.h
 *
 *  Created on: Aug 11, 2016
 *      Author: Agoston
 */

#ifndef SRC_MANIPULATOR_H_
#define SRC_MANIPULATOR_H_

#include <Constants.h>

class Manipulator
{
	public:
		Manipulator(Joystick *joystick, PowerDistributionPanel *pdp);
		virtual ~Manipulator();
		void update();
		void reset();

	protected:

	private:
		std::shared_ptr<Victor> motorControllers[ManipulatorMotors::NUM_MANIPULATOR_MOTORS];
		std::shared_ptr<AnalogPotentiometer> potentiometers[ManipulatorMotors::NUM_MANIPULATOR_MOTORS];
		uint32_t lastRunTimestamp;
		Joystick *joystick;
		PowerDistributionPanel *pdp;
};

#endif /* SRC_MANIPULATOR_H_ */
