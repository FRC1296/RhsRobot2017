/** \file
 * Example of subsystem task declaration.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control a given subsystem.
 *
 */

#ifndef COMPONENT_H
#define COMPONENT_H

/*
	A template class for creating new components
 */
#include <ComponentBase.h>			//For ComponentBase class
#include <pthread.h>
#include <string>
#include <CANTaLon.h>

//Robot
#include "WPILib.h"

class Hopper : public ComponentBase
{
public:
	Hopper();
	virtual ~Hopper();
	static void *StartTask(void *pThis, const char* szHopper, int iPriority)
	{
		pthread_setname_np(pthread_self(), szHopper);
		pthread_setschedprio(pthread_self(), iPriority);
		((Hopper *)pThis)->DoWork();
		return(NULL);
	}

private:
	void OnStateChange();
	void Run();
	CANTalon * pHopperMotor;
};

#endif			//COMPONENT_H
