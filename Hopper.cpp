/** \file
 * Example of subsystem task behavior.
 *
 * This class is derived from the standard Hopper base class and includes
 * initialization for the devices used to control a given subsystem.
 *
 * The task receives messages from the main robot class and implements behaviors
 * for a given subsystem.
 */

#include <math.h>
#include <assert.h>
#include <Hopper.h>

#include <string>
#include <iostream>
#include <algorithm>
#include <CANTaLon.h>
#include "Drivetrain.h"			//For the local header file
#include "RobotParams.h"
//Robot

Hopper::Hopper():
ComponentBase(HOPPER_TASKNAME, HOPPER_QUEUE, HOPPER_PRIORITY)
{
#ifndef USING_SOFTWARE_ROBOT
	pHopperMotor = new CANTalon(CAN_HOPPER_MOTOR);
#endif  // USING_SOFTWARE_ROBOT

	pTask = new std::thread(&Hopper::StartTask, this, HOPPER_TASKNAME, HOPPER_PRIORITY);
	wpi_assert(pTask);
};

Hopper::~Hopper()
{
	delete(pTask);
	delete(pHopperMotor);
};

void Hopper::OnStateChange()
{
	switch(localMessage.command)
	{
	case COMMAND_ROBOT_STATE_AUTONOMOUS:
		break;

	case COMMAND_ROBOT_STATE_TEST:
		break;

	case COMMAND_ROBOT_STATE_TELEOPERATED:
		break;

	case COMMAND_ROBOT_STATE_DISABLED:
		break;

	case COMMAND_ROBOT_STATE_UNKNOWN:
		break;

	default:
		break;
	}
}

void Hopper::Run()
{
#ifndef USING_SOFTWARE_ROBOT
	float StopMotor = pHopperMotor->GetOutputCurrent();
	SmartDashboard::PutNumber("Climber1 (1)", StopMotor);

	if(StopMotor >= 40)
	{
		pHopperMotor->Set(0);
	}

	else
#endif  // USING_SOFTWARE_ROBOT
	{
		switch(localMessage.command)			//Reads the message command
		{
		//TODO add command cases for Hopper
		case COMMAND_HOPPER_UP:
#ifndef USING_SOFTWARE_ROBOT
			pHopperMotor->Set(localMessage.params.hopper.HopUp);
#endif  // USING_SOFTWARE_ROBOT
			break;

		case COMMAND_HOPPER_DOWN:
#ifndef USING_SOFTWARE_ROBOT
			pHopperMotor->Set(localMessage.params.hopper.HopDown*-1);
#endif  // USING_SOFTWARE_ROBOT
			break;

		case COMMAND_HOPPER_STOP:
#ifndef USING_SOFTWARE_ROBOT
			pHopperMotor->Set(0);
#endif  // USING_SOFTWARE_ROBOT
			break;

		default: break;
		}
	}
};
