/** \file
 * Example of subsystem task behavior.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control a given subsystem.
 *
 * The task receives messages from the main robot class and implements behaviors
 * for a given subsystem.
 */

#include <ComponentBase.h>
#include <fuelIntake.h>
#include <RobotParams.h>
#include "WPILib.h"
#include <math.h>
#include <assert.h>
#include <string>
#include <iostream>
#include <algorithm>
using namespace std;
//Robot

fuelIntake::fuelIntake()
: ComponentBase(FUELINTAKE_TASKNAME, FUELINTAKE_QUEUE, FUELINTAKE_PRIORITY)
{
	//TODO: add member objects
	pTask = new std::thread(&fuelIntake::StartTask, this, FUELINTAKE_TASKNAME, FUELINTAKE_PRIORITY);
	wpi_assert(pTask);
};

fuelIntake::~fuelIntake()
{
	//TODO delete member objects
	delete(pTask);
};

void fuelIntake::OnStateChange()
{
};

void fuelIntake::Run()
{
	switch(localMessage.command)			//Reads the message command
	{
	//TODO add command cases for Component
		case COMMAND_COMPONENT_TEST:
			break;

		default:
			break;
		}
};
