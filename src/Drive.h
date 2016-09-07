/*
 * DozerDrive.h
 *
 *  Created on: Aug 4, 2016
 *      Author: Agoston
 */

#ifndef SRC_DRIVE_H_
#define SRC_DRIVE_H_

#include <Constants.h>

/*
 * Encoder counts per revolution = 7
 * x4 = 28 counts/rev
 * Gear ratio = 71
 * Counts per rotation of output shaft = 28 * 71 = 1988 counts/rev
 * Wheel circumference = 0.67 m/rev
 * Counts per meter = 2970 counts/m
 *
 * Motor no load speed = 75 rpm
 * Wheel no load velocity = (75 / 60) * 0.67 m = 0.84 m/s <- MAX SPEED CAPACITY = 2500 counts/s
 * Max speed from competition rules = 3 km/hr = 0.83 m/s <- CAPPED MAX SPEED = 2400 counts/s
 */

class Drive
{
	public:
		Drive(Joystick *controller, PowerDistributionPanel *pdp);
		virtual ~Drive();
		void update();
		void reset();

	private:
		std::shared_ptr<Victor> motorControllers[DriveMotors::NUM_DRIVE_MOTORS];
		std::shared_ptr<Encoder> encoders[DriveMotors::NUM_DRIVE_MOTORS];

		uint32_t lastEncoderVals[DriveMotors::NUM_DRIVE_MOTORS];
		float lastPowerVals[DriveMotors::NUM_DRIVE_MOTORS];
		const float maxSpeed = (float)(2400 * drivePeriod / 1000000); // Encoder counts per loop period
		const double kIntegral = 0.001;

		uint32_t lastRunTimestamp;
		Joystick *joystick;
		PowerDistributionPanel *pdp;
};

#endif /* SRC_DRIVE_H_ */
