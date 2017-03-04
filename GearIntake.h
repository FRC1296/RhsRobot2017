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

// define to use the encoder to open/close gear intake motor
#define  USE_GEARINTAKE_ENCODER

const int releaseEncoderPos = 3246;
const int holdEncoderPos = (releaseEncoderPos-2932-3246);
const float kPGainGear = 0.15;

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

	void OnStateChange();
	void Run();
};

#endif			//GEARINTAKE_H
