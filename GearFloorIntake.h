/** \file
 * Example of subsystem task declaration.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control a given subsystem.
 *
 */
#ifndef GEARFLOORINTAKE_H
#define GEARFLOORINTAKE_H

/**
	A template class for creating new components
 */
#include <ComponentBase.h>			//For ComponentBase class
#include <pthread.h>
#include <string>
#include <CANTalon.h>

//Robot
#include "WPILib.h"


class GearFloorIntake : public ComponentBase
{
public:
	GearFloorIntake();
	virtual ~GearFloorIntake();
	static void *StartTask(void *pThis, const char* szComponentName, int iPriority)
	{
		pthread_setname_np(pthread_self(), szComponentName);
		pthread_setschedprio(pthread_self(), iPriority);
		((GearFloorIntake *)pThis)->DoWork();
		return(NULL);
	}

private:
	CANTalon* pGearArmMotor;
	CANTalon* pGearIntakeMotor;
	PIDController* pArmPID;
	float fFloorPosition;
	float fDrivePosition;
	float fReleasePosition;

	const float fFromFloorToDrivePos = 250.0;
	const float fFromFloorToReleasePos = 125.0;
	const float fMaxIntakeSpeed = 1.0;
	const float fMaxArmCurrent = 40.0;

	const float fGearArmMotorIzone = 128.0;
	const float fGearArmMotorMaxRamp = 60.0;

	void OnStateChange();
	void Run();
};

#endif			//GEARINTAKE_H
