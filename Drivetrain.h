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

	CANTalon* pLeftMotor1;
	CANTalon* pLeftMotor2;
	CANTalon* pRightMotor1;
	CANTalon* pRightMotor2;
	ADXRS453Z *pGyro;
};

#endif			//DRIVETRAIN_H
