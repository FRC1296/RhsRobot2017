/** \file
 * Base class from which we derive our main robot class.
 *
 * The RhsRobotBase class is an extension to RobotBase and provides basic robot functionality.
 * This class sends state change messages, initializes the Smart Dashboard and calls the
 * work loop for this years robot.
 */

#include <assert.h>
#include <Autonomous.h>
#include <RhsRobotBase.h>			//For the local header file
#include <RobotParams.h>			//For various robot parameters
#include <sched.h>

//Built-In

#include "WPILib.h"			//For the RobotBase class, the watchdog, SmartDashboard, and the DriverStationLCD class

//Local

RhsRobotBase::RhsRobotBase()			//Constructor
{
	cpu_set_t  mask;
	//struct sched_param param;

	printf("\n\t\t%s \"%s\"\n\tVersion %s built %s at %s\n\n", ROBOT_NAME, ROBOT_NICKNAME, ROBOT_VERSION, __DATE__, __TIME__);

	// run our code on the second core

	CPU_ZERO(&mask);
	CPU_SET(1, &mask);
	sched_setaffinity(0, sizeof(mask), &mask);

	// what are our priority limits?

	previousRobotState = ROBOT_STATE_UNKNOWN;
	currentRobotState = ROBOT_STATE_UNKNOWN;
	loop = 0;			//Initializes the loop counter
}

RhsRobotBase::~RhsRobotBase()			//Destructor
{	
}

RobotOpMode RhsRobotBase::GetCurrentRobotState()			//Returns the current robot state
{
	return currentRobotState;
}

RobotOpMode RhsRobotBase::GetPreviousRobotState()			//Returns the previous robot state
{
	return previousRobotState;
}

bool RhsRobotBase::HasStateChanged()			//Returns if the robot state has just changed
{
	return (previousRobotState != currentRobotState);
}

int RhsRobotBase::GetLoop()			//Returns the loop number
{
	return loop;
}

void RhsRobotBase::StartCompetition()			//Robot's main function
{
	  DriverStation *pDS = &DriverStation::GetInstance();
	  LiveWindow* lw = LiveWindow::GetInstance();

	  HAL_Report(HALUsageReporting::kResourceType_Framework,
	             HALUsageReporting::kFramework_Simple);

	  NetworkTable::GetTable("LiveWindow")->GetSubTable("~STATUS~")->PutBoolean("LW Enabled", false);

      Init();		//Initialize the robot

	  // Tell the DS that the robot is ready to be enabled
	  HAL_ObserveUserProgramStarting();

	while(true)
	{
		pDS->WaitForData();

		//Checks the current state of the robot

		if(IsDisabled())
		{
			currentRobotState = ROBOT_STATE_DISABLED;
		}
		else if(IsEnabled() && IsAutonomous())
		{
			currentRobotState = ROBOT_STATE_AUTONOMOUS;
		}
		else if(IsEnabled() && IsOperatorControl())
		{
			currentRobotState = ROBOT_STATE_TELEOPERATED;
		}
		else if(IsEnabled() && IsTest())
		{
			currentRobotState = ROBOT_STATE_TEST;
		}
		else
		{
			currentRobotState = ROBOT_STATE_UNKNOWN;
		}

		if(HasStateChanged())			//Checks for state changes
		{
			switch(GetCurrentRobotState())
			{
			case ROBOT_STATE_DISABLED:
				printf("ROBOT_STATE_DISABLED\n");
				robotMessage.command = COMMAND_ROBOT_STATE_DISABLED;
				break;
			case ROBOT_STATE_AUTONOMOUS:
				printf("ROBOT_STATE_AUTONOMOUS\n");
				robotMessage.command = COMMAND_ROBOT_STATE_AUTONOMOUS;
				break;
			case ROBOT_STATE_TELEOPERATED:
				printf("ROBOT_STATE_TELEOPERATED\n");
				robotMessage.command = COMMAND_ROBOT_STATE_TELEOPERATED;
				break;
			case ROBOT_STATE_TEST:
				printf("ROBOT_STATE_TEST\n");
				robotMessage.command = COMMAND_ROBOT_STATE_TEST;
				break;
			case ROBOT_STATE_UNKNOWN:
				printf("ROBOT_STATE_UNKNOWN\n");
				robotMessage.command = COMMAND_ROBOT_STATE_UNKNOWN;
				break;
			}

			OnStateChange();			//Handles the state change
		}

		if(IsEnabled())
		{
			if((currentRobotState == ROBOT_STATE_TELEOPERATED) || 
					(currentRobotState == ROBOT_STATE_TEST) ||
					(currentRobotState == ROBOT_STATE_AUTONOMOUS))
			{
				Run();
			}

			if((currentRobotState == ROBOT_STATE_TEST))
			{
				lw->Run();
			}
		}

		previousRobotState = currentRobotState;

		++loop;		//Increment the loop counter
	}
}
