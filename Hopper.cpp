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
	pHopperMotor = new CANTalon(CAN_HOPPER_MOTOR);
	//TODO: add member objects
	pTask = new std::thread(&Hopper::StartTask, this, HOPPER_TASKNAME, HOPPER_PRIORITY);
	wpi_assert(pTask);
};

Hopper::~Hopper()
{
	delete (pHopperMotor);
	delete(pTask);
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
	switch(localMessage.command)			//Reads the message command
	{
	//TODO add command cases for Hopper
		SmartDashboard::PutNumber("Climber1 (1)", pHopperMotor->GetOutputCurrent());
	/*	float StopMotor = SmartDashboard::PutNumber("Climber1 (1)", pHopperMotor->GetOutputCurrent());
		if(StopMotor >= 40)
		{
			pHopperMotor->Set(0);
		}
		else
		{
			case COMMAND_HOPPER_UP:
				pHopperMotor->Set(localMessage.params.hopper.HopUp);
				break;

			case COMMAND_HOPPER_DOWN:
				pHopperMotor->Set(localMessage.params.hopper.HopDown*-1);
				break;

			case COMMAND_HOPPER_STOP:
				pHopperMotor->Set(0);
				break;
		}*/
	}
};
