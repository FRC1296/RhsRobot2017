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


typedef enum ArmPosition
{
	ARMPOS_UNKNOWN,
	ARMPOS_FLOOR,
	ARMPOS_DRIVE,
	ARMPOS_RELEASE,
	ARMPOS_LAST
} ArmPosition;


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
	//PIDController* pArmPID;
	float fFloorPosition;
	float fDrivePosition;
	float fReleasePosition;
	bool isInit = false;
	ArmPosition eCurrentPosition;

	const float fFromFloorToFloorPos = 0.1262;//-250.0;
	const float fFromFloorToDrivePos = 1.6195;//-250.0;
	const float fFromFloorToReleasePos = 1.031;//-125.0;
	const float fMaxIntakeSpeed = 1.0;
	const float fMaxArmCurrent = 40.0;

	const float fGearArmMotorIzone = 128.0;
	const float fGearArmMotorMaxRamp = 60.0;

	void OnStateChange();
	void Run();
	void InitGearArm();

};

#endif			//GEARINTAKE_H
