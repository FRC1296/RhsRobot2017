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
#ifndef USING_SOFTWARE_ROBOT
	pGearIntakeMotor = new CANTalon(CAN_GEARINTAKE_MOTOR);
	wpi_assert(pGearIntakeMotor);

	pGearIntakeMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
	pGearIntakeMotor->SetCurrentLimit(kMaxCurrentGear);
	pGearIntakeMotor->ConfigLimitMode(CANTalon::kLimitMode_SwitchInputsOnly);
	pGearIntakeMotor->SetControlMode(CANTalon::kPercentVbus);
	pGearIntakeMotor->Enable();
#endif  // USING_SOFTWARE_ROBOT

	pStateTimer = new Timer;

	pTask = new std::thread(&GearIntake::StartTask, this, GEARINTAKE_TASKNAME, GEARINTAKE_PRIORITY);
	wpi_assert(pTask);
};

GearIntake::~GearIntake()
{
	delete pTask;
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
#ifndef USING_SOFTWARE_ROBOT
			//pGearIntakeMotor->Set(localMessage.params.gear.GearRelease);
#endif  // USING_SOFTWARE_ROBOT
			break;

		case COMMAND_GEARINTAKE_HOLD:
#ifndef USING_SOFTWARE_ROBOT
			//pGearIntakeMotor->Set(-localMessage.params.gear.GearHold);
#endif  // USING_SOFTWARE_ROBOT
			break;

	    case COMMAND_GEARINTAKE_TENSION:
#ifndef USING_SOFTWARE_ROBOT
			//pGearIntakeMotor->Set(kHoldGearConstant);
#endif  // USING_SOFTWARE_ROBOT
			break;

	    case COMMAND_GEARINTAKE_STOP:
#ifndef USING_SOFTWARE_ROBOT
			//pGearIntakeMotor->Set(0.0);
#endif  // USING_SOFTWARE_ROBOT
			break;

	    case COMMAND_SYSTEM_MSGTIMEOUT:  //what should we do if we do not get a timely message?
			break;

		default:
			break;
		}

#ifndef USING_SOFTWARE_ROBOT
	// do not allow over current

	if(pGearIntakeMotor->GetOutputCurrent() > kMaxCurrentGear)
	{
		pGearIntakeMotor->Set(10.0);
	}
#endif  // USING_SOFTWARE_ROBOT

#ifndef USING_SOFTWARE_ROBOT
	switch(State)
	{
		case GearIntakeState_Hold:
			if (pGearIntakeMotor->IsRevLimitSwitchClosed())
			{
				pGearIntakeMotor->Set(0.0);
			}
			else
			{
				pGearIntakeMotor->Set(0.05);
			}

			if(localMessage.command == COMMAND_GEARINTAKE_RELEASE)
			{
				pStateTimer->Reset();
				pStateTimer->Start();
				State = GearIntakeState_HoldToRelease;
			}
			break;

		case GearIntakeState_Release:
			pGearIntakeMotor->Set(0.0);

			if(localMessage.command == COMMAND_GEARINTAKE_HOLD)
			{
				pStateTimer->Reset();
				pStateTimer->Start();
				State = GearIntakeState_ReleaseToHold;
			}
			break;

		case GearIntakeState_ReleaseToHold:
			if ((pStateTimer->Get() < 1.0) && (!pGearIntakeMotor->IsRevLimitSwitchClosed()))
			{
				pGearIntakeMotor->Set(-1.0);
			}
			else
			{
				State = GearIntakeState_Hold;
			}
			break;

		case GearIntakeState_HoldToRelease:
			if (pStateTimer->Get() < 1.0)
			{
				pGearIntakeMotor->Set(0.5);
			}
			else
			{
				State = GearIntakeState_Release;
			}
			break;

		default:
			break;
	}
#endif  // USING_SOFTWARE_ROBOT

};

