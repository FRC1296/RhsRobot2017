/** \file
 * Example of subsystem task behavior.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control a given subsystem.
 *
 * The task receives messages from the main robot class and implements behaviors
 * for a given subsystem.
 */

#include "WPILib.h"
#include "CheesyDrive.h"

CheezyLoop::CheezyLoop()
{
	bOutputEnabled = false;
	CheezyInit1296();  // initialize the cheezy drive code base

	pTask = new std::thread(&CheezyLoop::StartTask, this, CHEESY_TASKNAME, CHEESY_PRIORITY);
}

CheezyLoop::~CheezyLoop(){
	delete pTask;
}

void CheezyLoop::Run(void)
{

	 while(true)
	 {
		 Wait(0.005);

		if(bOutputEnabled)
		{
			 std::lock_guard<priority_recursive_mutex> sync(mutexData);
			 CheezyIterate1296(&currentGoal,
					 &currentPosition,
					 &currentOutput,
					 &currentStatus);
		}
		else
		{
			 std::lock_guard<priority_recursive_mutex> sync(mutexData);
			 CheezyIterate1296(&currentGoal,
					 &currentPosition,
					 NULL,
					 &currentStatus);
		}
	 }
}



void CheezyLoop::Update(const DrivetrainGoal &goal,
    const DrivetrainPosition &position,
    DrivetrainOutput &output,
    DrivetrainStatus &status,
	bool bEnabled)
{
	std::lock_guard<priority_recursive_mutex> sync(mutexData);

	bOutputEnabled = bEnabled;
	currentGoal = goal;
	currentPosition = position;
	output = currentOutput;
	status = currentStatus;
}



