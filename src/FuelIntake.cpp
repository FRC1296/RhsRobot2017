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
#include <FuelIntake.h>
#include <ComponentBase.h>
#include <RobotParams.h>
#include "WPILib.h"

//Robot

FuelIntake::FuelIntake()
: ComponentBase(FUELINTAKE_TASKNAME, FUELINTAKE_QUEUE, FUELINTAKE_PRIORITY)
{
	//TODO: add member objects
	pFuelIntakeMotor = new CANTalon(CAN_FUELINTAKE_MOTOR);
	pFuelIntakeMotor->SetVoltageRampRate(48.0);
	wpi_assert(pFuelIntakeMotor);
	pFuelIntakeMotor->ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
	pFuelIntakeMotor->SetControlMode(CANTalon::kPercentVbus);

	pTask = new std::thread(&FuelIntake::StartTask, this, FUELINTAKE_TASKNAME, FUELINTAKE_PRIORITY);
	wpi_assert(pTask);
};

FuelIntake::~FuelIntake()
{
	//TODO delete member objects
	delete(pTask);
	delete(pFuelIntakeMotor);
};

void FuelIntake::OnStateChange()
{
};

void FuelIntake::Run()
{
	SmartDashboard::PutNumber("Climber1 (1)", pFuelIntakeMotor->GetOutputCurrent());
	float StopMotor = SmartDashboard::PutNumber("Climber1 (1)", pFuelIntakeMotor->GetOutputCurrent());

	if (StopMotor>=40)
	{
		pFuelIntakeMotor->Set(0);
	}
	else
	{
		switch(localMessage.command)			//Reads the message command
		{
		//TODO add command cases for Component
		case COMMAND_COMPONENT_TEST:
			break;

		case COMMAND_FUELINTAKE_ON:
			pFuelIntakeMotor->Set(localMessage.params.fuel.fuelOn);
			break;

		case COMMAND_FUELINTAKE_STOP:
			pFuelIntakeMotor->Set(0);
			break;

		case COMMAND_SYSTEM_MSGTIMEOUT:
			break;

		default:
			break;
		}
	}
};
