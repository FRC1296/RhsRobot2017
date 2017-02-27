


/** \file
 * Example of subsystem task behavior.
 *
 * This class is derived from the standard Climber base class and includes
 * initialization for the devices used to control a given subsystem.
 *
 * The task receives messages from the main robot class and implements behaviors
 * for a given subsystem.
 */
#include <CANTalon.h>
#include <Climber.h>
#include <ComponentBase.h>
#include <RobotParams.h>
#include "WPILib.h"

//Robot

Climber::Climber()
: ComponentBase(CLIMBER_TASKNAME, CLIMBER_QUEUE, CLIMBER_PRIORITY)
{
	//TODO: add member objects
	pClimberMotor1 = new CANTalon(CAN_CLIMBER_MOTOR);
	pClimberMotor2 = new CANTalon(CAN_CLIMBER_MOTOR_SLAVE);

	pClimberMotor1->SetVoltageRampRate(48.0);
	pClimberMotor2->SetVoltageRampRate(48.0);

	wpi_assert(pClimberMotor1 && pClimberMotor2);

	pClimberMotor1->ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
	pClimberMotor1->SetControlMode(CANTalon::kPercentVbus);

	pClimberMotor2->ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
	pClimberMotor2->SetControlMode(CANTalon::kPercentVbus);

	pTask = new std::thread(&Climber::StartTask, this, CLIMBER_TASKNAME, CLIMBER_PRIORITY);
	wpi_assert(pTask);
};

Climber::~Climber()
{
	//TODO delete member objects
	delete(pTask);
	delete pClimberMotor1;
	delete pClimberMotor2;
};

void Climber::OnStateChange()
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

void Climber::Run()
{
	SmartDashboard::PutNumber("Climber1 (1)", pClimberMotor1->GetOutputCurrent());
	float StopMotor = SmartDashboard::PutNumber("Climber1 (1)", pClimberMotor1->GetOutputCurrent());

	if(StopMotor>=40)
	{
		pClimberMotor1->Set(0);
		pClimberMotor2->Set(0);
	}

	else
	{
		switch(localMessage.command)			//Reads the message command
		{
	//TODO add command cases for Climber
			case COMMAND_CLIMBER_UP:
				pClimberMotor1->Set(localMessage.params.climber.ClimbUp);
				pClimberMotor2->Set(localMessage.params.climber.ClimbUp*-1);
				break;

			case COMMAND_CLIMBER_DOWN:
				pClimberMotor1->Set(localMessage.params.climber.ClimbDown);
				pClimberMotor2->Set(localMessage.params.climber.ClimbDown*-1);
				break;

			case COMMAND_CLIMBER_STOP:
				pClimberMotor1->Set(0);
				pClimberMotor2->Set(0);
				break;

			case COMMAND_SYSTEM_MSGTIMEOUT:  // what should we do if we do not get a timely message?
				break;

			default:
				break;
		}
	}
};
