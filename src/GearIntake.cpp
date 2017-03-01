/** \file
* Example of subsystem task behavior.
*
* This class is derived from the standard Component base class and includes
* initialization for the devices used to control a given subsystem.
*
* The task receives messages from the main robot class and implements behaviors
* for a given subsystem.
*/

#include <CANTalon.h>
#include <GearIntake.h>
#include <ComponentBase.h>
#include <RobotParams.h>
#include "WPILib.h"

//Robot

GearIntake::GearIntake()
: ComponentBase(GEARINTAKE_TASKNAME, GEARINTAKE_QUEUE, GEARINTAKE_PRIORITY)
{
	pGearIntakeMotor = new CANTalon(CAN_GEARINTAKE_MOTOR);

	pGearIntakeMotor->SetVoltageRampRate(48.0);

	wpi_assert(pGearIntakeMotor);

	pGearIntakeMotor->ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
	pGearIntakeMotor->SetControlMode(CANTalon::kPercentVbus);

	pTask = new std::thread(&GearIntake::StartTask, this, GEARINTAKE_TASKNAME, GEARINTAKE_PRIORITY);
	wpi_assert(pTask);
};

GearIntake::~GearIntake()
{
	//TODO delete member objects
	delete(pTask);
	delete pGearIntakeMotor;
};

void GearIntake::OnStateChange()
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
};

void GearIntake::Run()
{
	switch(localMessage.command)			//Reads the message command
	{
		case COMMAND_GEARINTAKE_RELEASE:
						pGearIntakeMotor->Set(localMessage.params.gear.GearRelease);
						break;

		case COMMAND_GEARINTAKE_HOLD:
						pGearIntakeMotor->Set(-localMessage.params.gear.GearHold);
						break;

	    case COMMAND_GEARINTAKE_STOP:
						pGearIntakeMotor->Set(0);
						break;

	    case COMMAND_SYSTEM_MSGTIMEOUT:  //what should we do if we do not get a timely message?
						break;

		default:
			break;
		}
};

