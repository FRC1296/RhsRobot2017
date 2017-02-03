/** \file
 * Example of subsystem task declaration.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control a given subsystem.
 *
 */
#ifndef FUELINTAKE_H
#define FUELINTAKE_H

/**
	A template class for creating new components
 */
#include <ComponentBase.h>			//For ComponentBase class
#include <pthread.h>
#include <string>
#include <CANTalon.h>
#include <WPILib.h>
//Robot
#include "fuelIntake.h"


class fuelIntake : public ComponentBase
{
public:
	fuelIntake();
	virtual ~fuelIntake();
	static void *StartTask(void *pThis, const char* szComponentName, int iPriority)
	{
		pthread_setname_np(pthread_self(), szComponentName);
		pthread_setschedprio(pthread_self(), iPriority);
		((fuelIntake *)pThis)->DoWork();
		return(NULL);
	}

private:
	void OnStateChange();
	void Run();
	CANTalon* pfuelMotorOne;
	CANTalon* pfuelMothorOne;
};

#endif			//COMPONENT_H
