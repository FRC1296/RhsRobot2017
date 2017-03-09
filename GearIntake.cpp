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
	wpi_assert(pGearIntakeMotor);

	pGearIntakeMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
	pGearIntakeMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
	pGearIntakeMotor->SetPID(kPGainGear, 0.0, 0.0);

#ifdef USE_GEARINTAKE_ENCODER
	pGearIntakeMotor->SetControlMode(CANTalon::kPosition);
	pGearIntakeMotor->Set(holdEncoderPos);
#else
	pGearIntakeMotor->SetControlMode(CANTalon::kPercentVbus);
#endif
	pGearIntakeMotor->Enable();

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
#ifdef USE_GEARINTAKE_ENCODER
			pGearIntakeMotor->Set(localMessage.params.gear.GearRelease);
#else
			pGearIntakeMotor->Set(releaseEncoderPos);
#endif
			break;

		case COMMAND_GEARINTAKE_HOLD:
#ifdef USE_GEARINTAKE_ENCODER
			pGearIntakeMotor->Set(-localMessage.params.gear.GearHold);
#else
			pGearIntakeMotor->Set(holdEncoderPos);
#endif
			break;

	    case COMMAND_GEARINTAKE_STOP:
#ifdef USE_GEARINTAKE_ENCODER
	    	// no need to turn it off, the servo will do that (one hopes)
#else
			pGearIntakeMotor->Set(0);
#endif
			break;

	    case COMMAND_SYSTEM_MSGTIMEOUT:  //what should we do if we do not get a timely message?
			break;

		default:
			break;
		}

	SmartDashboard::PutNumber("gear encoder", pGearIntakeMotor->GetPulseWidthPosition());
	SmartDashboard::PutNumber("gear release", releaseEncoderPos);
	SmartDashboard::PutNumber("gear hold", holdEncoderPos);
};

