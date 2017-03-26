


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
#ifndef USING_SOFTWARE_ROBOT
	pClimberMotor1 = new CANTalon(CAN_CLIMBER_MOTOR);
	pClimberMotor2 = new CANTalon(CAN_CLIMBER_MOTOR_SLAVE);

	pClimberMotor1->SetVoltageRampRate(48.0);
	pClimberMotor2->SetVoltageRampRate(48.0);

	wpi_assert(pClimberMotor1 && pClimberMotor2);

	pClimberMotor1->ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
	pClimberMotor1->SetControlMode(CANTalon::kPercentVbus);

	pClimberMotor2->ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
	pClimberMotor2->SetControlMode(CANTalon::kPercentVbus);
#endif // USING_SOFTWARE_ROBOT
	pAutoTimer = new Timer();
	inAuto = false;
	autoClimb = false;

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
				inAuto = true;
				break;

			case COMMAND_ROBOT_STATE_TEST:
				inAuto = false;
				break;

			case COMMAND_ROBOT_STATE_TELEOPERATED:
				inAuto = false;
				break;

			case COMMAND_ROBOT_STATE_DISABLED:
				inAuto = false;
				break;

			case COMMAND_ROBOT_STATE_UNKNOWN:
				inAuto = false;
				break;

			default:
				break;
		}
};

void Climber::Run()
{
#ifndef USING_SOFTWARE_ROBOT
	float StopMotor;

	if(iLoop % 10 == 0)
	{
		StopMotor = pClimberMotor1->GetOutputCurrent();
		SmartDashboard::PutNumber("Climber1 (1)", StopMotor);

		if(StopMotor>=40.0)
		{
			pClimberMotor1->Set(0.0);
			pClimberMotor2->Set(0.0);
		}
	}

	else
#endif // USING_SOFTWARE_ROBOT
	{
		switch(localMessage.command)			//Reads the message command
		{
	//TODO add command cases for Climber
			case COMMAND_CLIMBER_UP:
#ifndef USING_SOFTWARE_ROBOT
				pClimberMotor1->Set(localMessage.params.climber.ClimbUp);
				pClimberMotor2->Set(localMessage.params.climber.ClimbUp*-1);
#endif // USING_SOFTWARE_ROBOT
				break;

			case COMMAND_CLIMBER_DOWN:
#ifndef USING_SOFTWARE_ROBOT
				pClimberMotor1->Set(localMessage.params.climber.ClimbDown);
				pClimberMotor2->Set(localMessage.params.climber.ClimbDown*-1);
#endif // USING_SOFTWARE_ROBOT
				break;

			case COMMAND_CLIMBER_STOP:
#ifndef USING_SOFTWARE_ROBOT
				pClimberMotor1->Set(0);
				pClimberMotor2->Set(0);
#endif // USING_SOFTWARE_ROBOT
				break;

			case COMMAND_AUTO_CLIMBER:
				autoClimb= true;
				AutoClimber(0.50);
				autoClimb= false;
				break;

			case COMMAND_SYSTEM_MSGTIMEOUT:  // what should we do if we do not get a timely message?
				break;

			default:
				break;
		}
	}
}


void Climber::AutoClimber(float time) {
	pAutoTimer->Reset();
	pAutoTimer->Start();

	if (autoClimb) {
		while (true) {
			if ((pAutoTimer->Get() < time) && inAuto) {
#ifndef USING_SOFTWARE_ROBOT
				pClimberMotor1->Set(localMessage.params.climber.ClimbUp);
				pClimberMotor2->Set(-localMessage.params.climber.ClimbUp);
#endif // USING_SOFTWARE_ROBOT
			}
			else
			{
				break;
			}

			Wait(0.005);
		}
#ifndef USING_SOFTWARE_ROBOT
		pClimberMotor1->Set(0);
		pClimberMotor2->Set(0);
#endif // USING_SOFTWARE_ROBOT
	}
}
