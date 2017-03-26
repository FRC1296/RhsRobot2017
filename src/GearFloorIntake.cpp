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

	//pGearArmMotor->SetInverted(true);
	//pGearArmMotor->SetP(.1);
	//pGearArmMotor->SetD(0);
	/*	SmartDashboard::PutString("SETTING:", "INIT");
	pGearArmMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
	pGearArmMotor->ConfigPeakOutputVoltage(+6.0,-6.0);
	pGearArmMotor->SetCloseLoopRampRate(10000);
	pGearArmMotor->SetVoltageRampRate(10000);
	pGearArmMotor->SetTalonControlMode(CANTalon::kPositionMode);
	pGearArmMotor->SetP(.5);
	pGearArmMotor->SetD(.5);*/



	/*//pGearArmMotor->SetControlMode(CANTalon::kPercentVbus);
	pGearArmMotor->SetSensorDirection(false);
	pGearArmMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
	pGearArmMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
	//pGearArmMotor->SetInverted(false);
	pGearArmMotor->SetIzone(fGearArmMotorIzone);
	//pGearArmMotor->SetCloseLoopRampRate(fGearArmMotorMaxRamp);
	pGearArmMotor->SetVoltageRampRate(0);
	pGearArmMotor->SelectProfileSlot(0);
	pGearArmMotor->SetControlMode(CANTalon::kPosition);

	//pGearArmMotor->ConfigSoftPositionLimits(+15,-15);


	//pGearArmMotor->SetP(.01);

	/*pArmPID = new PIDController(.0010, 0.0, 0.0, pGearArmMotor, pGearArmMotor, .05);
	wpi_assert(pArmPID);*/;

	/*	fFloorPosition = pGearArmMotor->GetPulseWidthPosition()/4096;

	//pGearArmMotor->SetPosition(0);
	fDrivePosition = fFloorPosition - .84375;
	fReleasePosition  = fFloorPosition + fFromFloorToReleasePos/4096;
	/*pArmPID->SetSetpoint(fFloorPosition);
	pArmPID->Enable();*/
	eCurrentPosition = ARMPOS_FLOOR;

	pTask = new std::thread(&GearFloorIntake::StartTask, this, GEARFLOORINTAKE_TASKNAME, GEARFLOORINTAKE_PRIORITY);
	wpi_assert(pTask);
};

void GearFloorIntake::InitGearArm()
{
	SmartDashboard::PutString("SETTING:", "ZERO");
	pGearArmMotor->SetControlMode(CANTalon::kPercentVbus);
	//pGearArmMotor->SetCurrentLimit(5);
	pGearArmMotor->Set(.5);
	//pGearArmMotor->EnableCurrentLimit(true);
	Wait(0.25);
	while(pGearArmMotor->GetOutputCurrent() < 3)
	{
		SmartDashboard::PutNumber("GearCurrent", pGearArmMotor->GetOutputCurrent());
		SmartDashboard::PutNumber("GearVelocity", pGearArmMotor->GetEncVel());
	}
	pGearArmMotor->Set(0);

	Wait(0.25);

	pGearArmMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Absolute);
	pGearArmMotor->ConfigPeakOutputVoltage(+6.0,-6.0);
	pGearArmMotor->SetCloseLoopRampRate(10000);
	pGearArmMotor->SetVoltageRampRate(10000);
	pGearArmMotor->SetTalonControlMode(CANTalon::kPositionMode);
	pGearArmMotor->SetP(.5);
	pGearArmMotor->SetD(.5);


	fFloorPosition = (pGearArmMotor->GetPulseWidthPosition()*1.0)/4096;
	fDrivePosition = fFloorPosition - fFromFloorToDrivePos;
	fReleasePosition  = fFloorPosition - fFromFloorToReleasePos;
	fFloorPosition = fFloorPosition - fFromFloorToFloorPos;

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
		InitGearArm();
		break;

	case COMMAND_ROBOT_STATE_TEST:
		break;

	case COMMAND_ROBOT_STATE_TELEOPERATED:
		if(!isInit)InitGearArm();
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
	float motorspeed = pGearArmMotor->GetOutputVoltage();
	//double pval = pGearArmMotor->GetP();

	SmartDashboard::PutNumber("Arm Current", fStopMotor);
	SmartDashboard::PutNumber("Arm Position", fPosition/4096);
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

	if (pGearIntakeMotor->IsRevLimitSwitchClosed()) {
		;
	}
	switch(localMessage.command)			//Reads the message command
	{
	case COMMAND_GEARFLOORINTAKE_INTAKEPOS:
		if(!isInit) InitGearArm();
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

