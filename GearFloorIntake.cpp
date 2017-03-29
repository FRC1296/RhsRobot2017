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
	wpi_assert(pGearIntakeMotor);

	pGearIntakeMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
	pGearIntakeMotor->SetControlMode(CANTalon::kPercentVbus);
	pGearIntakeMotor->Enable();

	pGearArmMotor = new CANTalon(CAN_FLOORINTAKEARM_MOTOR);
	wpi_assert(pGearArmMotor);
	isInit = false;

	//pGearArmMotor->SetControlMode(CANTalon::kPercentVbus);
	//pGearArmMotor->SetCurrentLimit(5);
	//pGearArmMotor->Set(.5);
	//pGearArmMotor->EnableCurrentLimit(true);

	pGearArmMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
	pGearArmMotor->ConfigPeakOutputVoltage(+6.0,-6.0);
	pGearArmMotor->SetCloseLoopRampRate(10000);
	pGearArmMotor->SetVoltageRampRate(10000);
	pGearArmMotor->Set(0.0);
	pGearArmMotor->SetP(.5);
	pGearArmMotor->SetI(.005);
	pGearArmMotor->SetD(.57);
	pGearArmMotor->SetControlMode(CANTalon::kPercentVbus);

	eCurrentPosition = ARMPOS_FLOOR;

	pTask = new std::thread(&GearFloorIntake::StartTask, this, GEARFLOORINTAKE_TASKNAME, GEARFLOORINTAKE_PRIORITY);
	wpi_assert(pTask);
};

void GearFloorIntake::InitGearArm()
{
	SmartDashboard::PutString("SETTING:", "ZERO");
	pGearArmMotor->SetTalonControlMode(CANTalon::kPositionMode);

	fDrivePosition =  (pGearArmMotor->GetPulseWidthPosition()*1.0)/4096;
	fReleasePosition = fDrivePosition + fFromRobotToReleasePos;
	fFloorPosition = fDrivePosition + fFromRobotToFloorPos;

	pGearArmMotor->Set(fReleasePosition);
	eCurrentPosition = ARMPOS_RELEASE;
	isInit = true;
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
			if(!isInit)
			{
				InitGearArm(); // set gear intake arm zero position
			}
			break;

		case COMMAND_ROBOT_STATE_TEST:
			if(!isInit)
			{
				InitGearArm(); // set gear intake arm zero position
			}
			break;

		case COMMAND_ROBOT_STATE_TELEOPERATED:
			if(!isInit)
			{
				InitGearArm(); // set gear intake arm zero position
			}
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
	float fStopMotor;
	float fPosition;
	float motorspeed;

	if(iLoop % 10 == 0)
	{
		fStopMotor = pGearArmMotor->GetOutputCurrent();
		fPosition = pGearArmMotor->GetPulseWidthPosition();
		motorspeed = pGearArmMotor->GetOutputVoltage();
		//double pval = pGearArmMotor->GetP();

		SmartDashboard::PutNumber("Arm Current", fStopMotor);
		SmartDashboard::PutNumber("Arm Position", fPosition/4096.0);
		SmartDashboard::PutNumber("FLOOR POS", fFloorPosition);
		SmartDashboard::PutNumber("DRIVE POS", fDrivePosition);
		SmartDashboard::PutNumber("RELEASE POS", fReleasePosition);
		SmartDashboard::PutNumber("Speed:", motorspeed);

		// stay here if the current is exceeded

		if(fStopMotor >= fMaxArmCurrent)
		{
			//pGearArmMotor->SetPosition(0);
			pGearArmMotor->Set(fPosition);
		}

		if(eCurrentPosition == ARMPOS_FLOOR && pGearIntakeMotor->IsRevLimitSwitchClosed()) {
			pGearArmMotor->Set(fDrivePosition);
			eCurrentPosition = ARMPOS_DRIVE;
			SmartDashboard::PutString("SETTING:", "DRIVE POS (1)");
		}

		SmartDashboard::PutBoolean("Gear?", pGearIntakeMotor->IsRevLimitSwitchClosed());
	}



	switch(localMessage.command)			//Reads the message command
	{
	case COMMAND_MACRO_HANGGEAR:
		pGearIntakeMotor->Set(0.4);
		Wait(0.200);
		pGearIntakeMotor->Set(0.0);
		pGearArmMotor->Set(fFloorPosition);
		eCurrentPosition = ARMPOS_FLOOR;
		Wait(0.400);
		pGearArmMotor->Set(fReleasePosition);
		eCurrentPosition = ARMPOS_RELEASE;
		Wait(0.100);
		ClearMessages();
		break;

	case COMMAND_GEARFLOORINTAKE_INTAKEPOS:
		pGearArmMotor->Set(fFloorPosition);
		eCurrentPosition = ARMPOS_FLOOR;
		SmartDashboard::PutString("SETTING:", "INTAKE POS (0)");
		break;

	case COMMAND_GEARFLOORINTAKE_DRIVEPOS:
		pGearArmMotor->Set(fDrivePosition);
		eCurrentPosition = ARMPOS_DRIVE;
		SmartDashboard::PutString("SETTING:", "DRIVE POS (1)");
		break;

	case COMMAND_GEARFLOORINTAKE_RELEASEPOS:
		pGearArmMotor->Set(fReleasePosition);
		eCurrentPosition = ARMPOS_RELEASE;
		SmartDashboard::PutString("SETTING:", "SCORE POS(2)");
		break;

	case COMMAND_GEARFLOORINTAKE_NEXTPOS:
		if(eCurrentPosition == ARMPOS_FLOOR)
		{
			pGearArmMotor->Set(fDrivePosition);
			eCurrentPosition = ARMPOS_DRIVE;
		}
		else if(eCurrentPosition == ARMPOS_DRIVE)
		{
			//pGearArmMotor->Set(fReleasePosition);

			pGearArmMotor->Set(fReleasePosition);
			eCurrentPosition = ARMPOS_RELEASE;
		}
		else if(eCurrentPosition == ARMPOS_RELEASE)
		{
			pGearArmMotor->Set(fReleasePosition);
			eCurrentPosition = ARMPOS_RELEASE;
		}
		else
		{
			pGearArmMotor->Set(fDrivePosition);
			eCurrentPosition = ARMPOS_DRIVE;
		}
		break;

	case COMMAND_GEARFLOORINTAKE_PREVPOS:
		if(eCurrentPosition == ARMPOS_FLOOR)
		{
			pGearArmMotor->Set(fFloorPosition);
			eCurrentPosition = ARMPOS_FLOOR;
		}
		else if(eCurrentPosition == ARMPOS_DRIVE)
		{
			pGearArmMotor->Set(fFloorPosition);
			eCurrentPosition = ARMPOS_FLOOR;
		}
		else if(eCurrentPosition == ARMPOS_RELEASE)
		{
			pGearArmMotor->Set(fDrivePosition);
			eCurrentPosition = ARMPOS_DRIVE;
		}
		else
		{
			pGearArmMotor->Set(fDrivePosition);
			eCurrentPosition = ARMPOS_DRIVE;
		}
		break;

	case COMMAND_GEARFLOORINTAKE_PULLIN:
		if (eCurrentPosition == ARMPOS_DRIVE)
		{
			if(localMessage.params.floor.fSpeed > (fMaxIntakeSpeed / 3.0)) {

				pGearIntakeMotor->Set(fMaxIntakeSpeed/3.0);
			}
			else {
				pGearIntakeMotor->Set(localMessage.params.floor.fSpeed);
			}
		}
		else if(localMessage.params.floor.fSpeed > fMaxIntakeSpeed)
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
		pGearIntakeMotor->Set(0.0);
		break;

	case COMMAND_SYSTEM_MSGTIMEOUT:  //what should we do if we do not get a timely message?
		break;

	default:
		break;
	}

};

