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
#include <GearFloorIntake.h>
#include <ComponentBase.h>
#include <RobotParams.h>
#include "WPILib.h"

//Robot

GearFloorIntake::GearFloorIntake()
: ComponentBase(GEARFLOORINTAKE_TASKNAME, GEARFLOORINTAKE_QUEUE, GEARFLOORINTAKE_PRIORITY)
{
	pGearIntakeMotor = new CANTalon(CAN_FLOORINTAKEROLLER_MOTOR);
	wpi_assert(pGearArmMotor);

	pGearIntakeMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
	pGearIntakeMotor->SetControlMode(CANTalon::kPercentVbus);
	pGearIntakeMotor->Enable();

	pGearArmMotor = new CANTalon(CAN_FLOORINTAKEARM_MOTOR );
	wpi_assert(pGearArmMotor);
	pGearArmMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
	pGearArmMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
	pGearArmMotor->SetInverted(true);
	pGearArmMotor->SetIzone(fGearArmMotorIzone);
	pGearArmMotor->SetCloseLoopRampRate(fGearArmMotorMaxRamp);
	pGearArmMotor->SetControlMode(CANTalon::kPercentVbus);

	pArmPID = new PIDController(.0010, 0.0, 0.0, pGearArmMotor, pGearArmMotor, .05);
	wpi_assert(pArmPID);
	fFloorPosition = pGearIntakeMotor->GetPulseWidthPosition();
	fDrivePosition = fFloorPosition + fFromFloorToDrivePos;
	fReleasePosition  = fFloorPosition + fFromFloorToReleasePos;
	pArmPID->SetSetpoint(fFloorPosition);
	pArmPID->Enable();

	pTask = new std::thread(&GearFloorIntake::StartTask, this, GEARFLOORINTAKE_TASKNAME, GEARFLOORINTAKE_PRIORITY);
	wpi_assert(pTask);
};

GearFloorIntake::~GearFloorIntake()
{
	delete pTask;
};

void GearFloorIntake::OnStateChange()
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

void GearFloorIntake::Run()
{
	float fStopMotor = pGearArmMotor->GetOutputCurrent();
	float fPosition = pGearArmMotor->GetPulseWidthPosition();

	SmartDashboard::PutNumber("Arm Current", fStopMotor);
	SmartDashboard::PutNumber("Arm Position", fPosition);

	// stay here if the current is exceeded

	if(fStopMotor >= fMaxArmCurrent)
	{
		pArmPID->SetSetpoint(fPosition);
	}

	switch(localMessage.command)			//Reads the message command
	{
		case COMMAND_GEARFLOORINTAKE_INTAKEPOS:
			pArmPID->SetSetpoint(fFloorPosition);
			break;

	    case COMMAND_GEARFLOORINTAKE_DRIVEPOS:
	    	pArmPID->SetSetpoint(fDrivePosition);
			break;

		case COMMAND_GEARFLOORINTAKE_RELEASEPOS:
			pArmPID->SetSetpoint(fReleasePosition);
			break;

	    case COMMAND_GEARFLOORINTAKE_PULLIN:
	    	if(localMessage.params.floor.fSpeed > fMaxIntakeSpeed)
	    	{
		    	pGearIntakeMotor->Set(fMaxIntakeSpeed);
	    	}
	    	else
	    	{
		    	pGearIntakeMotor->Set(localMessage.params.floor.fSpeed);
	    	}
			break;

	    case COMMAND_GEARFLOORINTAKE_PUSHOUT:
	    	if(localMessage.params.floor.fSpeed > fMaxIntakeSpeed)
	    	{
		    	pGearIntakeMotor->Set(-fMaxIntakeSpeed);
	    	}
	    	else
	    	{
		    	pGearIntakeMotor->Set(-localMessage.params.floor.fSpeed);
	    	}
			break;

	    case COMMAND_GEARFLOORINTAKE_STOP:
			break;

	    case COMMAND_SYSTEM_MSGTIMEOUT:  //what should we do if we do not get a timely message?
			break;

		default:
			break;
		}

};

