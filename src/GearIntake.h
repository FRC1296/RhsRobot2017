/** \file
 * Example of subsystem task declaration.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control a given subsystem.
 *
 */
#ifndef GEARINTAKE_H
#define GEARINTAKE_H

/**
	A template class for creating new components
 */
#include <ComponentBase.h>			//For ComponentBase class
#include <pthread.h>
#include <string>
#include <CANTalon.h>

//Robot
#include "WPILib.h"

const float kPGainGear = 0.10;
const float kMaxCurrentGear = 20.0;
const float kHoldGearConstant = -0.025;

typedef enum GearIntakeState
{
	GearIntakeState_Hold,
	GearIntakeState_Release,
	GearIntakeState_ReleaseToHold,
	GearIntakeState_HoldToRelease,
	GearIntakeState_Last
} GearIntakeState;

class GearIntake : public ComponentBase
{
public:
	GearIntake();
	virtual ~GearIntake();
	static void *StartTask(void *pThis, const char* szComponentName, int iPriority)
	{
		pthread_setname_np(pthread_self(), szComponentName);
		pthread_setschedprio(pthread_self(), iPriority);
		((GearIntake *)pThis)->DoWork();
		return(NULL);
	}

private:
	CANTalon* pGearIntakeMotor;
	GearIntakeState State = GearIntakeState_Hold;
	Timer *pStateTimer;

	void OnStateChange();
	void Run();
};

#endif			//GEARINTAKE_H
