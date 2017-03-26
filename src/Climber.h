/** \file
 * Example of subsystem task declaration.
 *
 * This class is derived from the standard Climber base class and includes
 * initialization for the devices used to control a given subsystem.
 *
 */
#ifndef Climber_H
#define Climber_H

/**
	A template class for creating new Climbers
 */
#include <ComponentBase.h>			//For ClimberBase class
#include <pthread.h>
#include <string>
#include <CANTalon.h>

//Robot
#include "WPILib.h"


class Climber : public ComponentBase
{
public:
	Climber();
	virtual ~Climber();
	static void *StartTask(void *pThis, const char* szClimberName, int iPriority)
	{
		pthread_setname_np(pthread_self(), szClimberName);
		pthread_setschedprio(pthread_self(), iPriority);
		((Climber *)pThis)->DoWork();
		return(NULL);
	}

private:
	CANTalon* pClimberMotor1;
	CANTalon* pClimberMotor2;
	Timer* pAutoTimer;

	bool autoClimb;
	bool inAuto;

	void OnStateChange();
	void Run();
	void AutoClimber(float);
};

#endif			//Climber_H
