/*
 * Safety.h
 *
 *  Created on: Sep 3, 2016
 *      Author: Agoston
 */

#ifndef SRC_SAFETY_H_
#define SRC_SAFETY_H_

#include <Constants.h>

class Safety
{
	public:
		Safety(Joystick *joystick, PowerDistributionPanel *pdp);
		virtual ~Safety();
		void update();
		void reset();

	private:
		std::shared_ptr<DigitalOutput> powerRelay[2];
		uint32_t lastRunTimestamp;
		Joystick *joystick;
		PowerDistributionPanel *pdp;
};

#endif /* SRC_SAFETY_H_ */
