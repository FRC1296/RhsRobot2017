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
#include <CANTAlon.h>

//Robot
#include "WPILib.h"


class FuelIntake : public ComponentBase
{
public:
	FuelIntake();
	virtual ~FuelIntake();
	static void *StartTask(void *pThis, const char* szFuelIntakeName, int iPriority)
	{
		pthread_setname_np(pthread_self(), szFuelIntakeName);
		pthread_setschedprio(pthread_self(), iPriority);
		((FuelIntake *)pThis)->DoWork();
		return(NULL);
	}

private:
	CANTalon* pFuelIntakeMotor;

	void OnStateChange();
	void Run();
};

#endif			//FUELINTAKE_H
