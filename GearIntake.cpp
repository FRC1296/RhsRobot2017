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
	pGearIntakeMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	pGearIntakeMotor->SetPID(kPGainGear, 0.0, 0.0);
	pGearIntakeMotor->SetCurrentLimit(kMaxCurrentGear);
	pGearIntakeMotor->ConfigLimitMode(CANTalon::kLimitMode_SwitchInputsOnly);

#ifdef USE_GEARINTAKE_ENCODER
	pGearIntakeMotor->SetControlMode(CANTalon::kPosition);
	holdEncoderPos = pGearIntakeMotor->GetPulseWidthPosition();
	releaseEncoderPos = holdEncoderPos + releaseEncoderPos;
	pGearIntakeMotor->Set(holdEncoderPos);
#else
	pGearIntakeMotor->SetControlMode(CANTalon::kPercentVbus);
#endif
	pGearIntakeMotor->Enable();
#endif  // USING_SOFTWARE_ROBOT
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
#ifdef USE_GEARINTAKE_ENCODER
			pGearIntakeMotor->Set(releaseEncoderPos);
#else
			pGearIntakeMotor->Set(localMessage.params.gear.GearRelease);
#endif
#endif  // USING_SOFTWARE_ROBOT
			break;

		case COMMAND_GEARINTAKE_HOLD:
#ifndef USING_SOFTWARE_ROBOT
#ifdef USE_GEARINTAKE_ENCODER
			pGearIntakeMotor->Set(holdEncoderPos);
#else
			pGearIntakeMotor->Set(-localMessage.params.gear.GearHold);
#endif
#endif  // USING_SOFTWARE_ROBOT
			break;

	    case COMMAND_GEARINTAKE_TENSION:
#ifndef USING_SOFTWARE_ROBOT
#ifdef USE_GEARINTAKE_ENCODER
	    	// no need to turn it off, the servo will do that (one hopes)
#else
			pGearIntakeMotor->Set(kHoldGearConstant);
#endif
#endif  // USING_SOFTWARE_ROBOT
			break;

	    case COMMAND_GEARINTAKE_STOP:
#ifndef USING_SOFTWARE_ROBOT
#ifdef USE_GEARINTAKE_ENCODER
	    	// no need to turn it off, the servo will do that (one hopes)
#else
			pGearIntakeMotor->Set(0.0);
#endif
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

//	SmartDashboard::PutNumber("gear encoder", pGearIntakeMotor->GetPulseWidthPosition());/
//	SmartDashboard::PutNumber("gear release", releaseEncoderPos);
//	SmartDashboard::PutNumber("gear hold", holdEncoderPos);
	SmartDashboard::PutNumber("gear current", pGearIntakeMotor->GetOutputCurrent());
#endif  // USING_SOFTWARE_ROBOT
};

