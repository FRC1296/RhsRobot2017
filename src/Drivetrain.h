/** \file
 * Definitions of class to control the drive train.
 *
 * This classes is derived from the standard Component base class and includes
 * definitions for the devices used to control the pallet jack wheels.
 */

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

//Robot
#include <WPILib.h>
#include <CANTaLon.h>

#include <ComponentBase.h>			//For ComponentBase class
#include <pthread.h>

#include "ADXRS453Z.h"

// constants used to tune TALONS

// measured 546 left and 534 right on the ground,
// 641 left and 562 right on blocks on 21 March 2016

const float FULLSPEED_FROMTALONS = 	1422.22;	// measured on the robot in RPMs
const float TALON_FTERM_L = 		0.21;	// From CTRE manual, section 12.4
const float TALON_PTERM_L = 		(TALON_FTERM_L / 5.0);
const float TALON_ITERM_L = 		(TALON_PTERM_L / 10.0);
const float TALON_DTERM_L = 		(TALON_PTERM_L / 5.0);
const float TALON_FTERM_R = 		0.24;	// From CTRE manual, section 12.4
const float TALON_PTERM_R = 		(TALON_FTERM_R / 5.0);
const float TALON_ITERM_R = 		(TALON_PTERM_R / 10.0);
const float TALON_DTERM_R = 		(TALON_PTERM_R / 5.0);
const float TALON_MAXRAMP =			60;		// 200ms
const float TALON_IZONE	=			128;
const float TALON_COUNTSPERREV =	360;	// from CTRE docs
const float REVSPERFOOT = (3.141519 * 6.0 / 12.0);
const double METERS_PER_COUNT = (REVSPERFOOT * 0.3048 / (double)TALON_COUNTSPERREV);


class CheezyLoop;

class Drivetrain : public ComponentBase
{
public:
	Drivetrain();
	~Drivetrain();

	static void *StartTask(void *pThis, const char* szComponentName, int iPriority)
	{
		pthread_setname_np(pthread_self(), szComponentName);
		pthread_setschedprio(pthread_self(), iPriority);
		((Drivetrain *)pThis)->DoWork();
		return(NULL);
	}

private:
	void OnStateChange();
	void Run();
	void RunCheezyDrive(bool, float, float, bool);

	CANTalon* pLeftMotor;
	CANTalon* pRightMotor;
	CANTalon* pLeftMotorSlave;
	CANTalon* pRightMotorSlave;

	ADXRS453Z *pGyro;
	float fBatteryVoltage;
	bool bUnderServoControl;
	CheezyLoop *pCheezy;
};

#endif			//DRIVETRAIN_H
