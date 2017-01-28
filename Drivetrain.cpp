/** \file
 * Implementation of class to drive the pallet jack.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control the pallet jack's wheels.
 *
 * The task receives messages form the main robot class and runs the wheels.
 * Special commands use a pGyro and quadrature encoder to drive straight X feet
 * or to turn X degrees.
 *
 * Motor orientations:
 * left +
 * right -
 */

#include <math.h>
#include <assert.h>
#include <ComponentBase.h>

#include <string>
#include <iostream>
#include <algorithm>

#include "Drivetrain.h"			//For the local header file
#include "RobotParams.h"


using namespace std;

Drivetrain::Drivetrain() :
		ComponentBase(DRIVETRAIN_TASKNAME, DRIVETRAIN_QUEUE,
				DRIVETRAIN_PRIORITY) {
	// create all the objects used in this thread

	pLeftMotor = new CANTalon(CAN_DRIVETRAIN_LEFT_MOTOR);
	pRightMotor = new CANTalon(CAN_DRIVETRAIN_RIGHT_MOTOR);
	wpi_assert(pLeftMotor && pRightMotor);

	// initialize the motor on the left side

	pLeftMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
	pLeftMotor->SetControlMode(CANTalon::kPercentVbus);

	// initialize the motor on the right side

	pRightMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
	pRightMotor->SetControlMode(CANTalon::kPercentVbus);

	pGyro = new ADXRS453Z();
	wpi_assert(pGyro);

	pTask = new std::thread(&Drivetrain::StartTask, this,
			DRIVETRAIN_TASKNAME, DRIVETRAIN_PRIORITY);
	wpi_assert(pTask);
}

Drivetrain::~Drivetrain()			//Destructor
{
	// clean up here, delete things in reverse order

	delete pTask;
	delete pLeftMotor;
	delete pRightMotor;
	delete pGyro;
}

void Drivetrain::OnStateChange()
{
	// do we need to do anything when the robot state changes?

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
}

void Drivetrain::Run() {

	switch(localMessage.command)
	{
		case COMMAND_DRIVETRAIN_DRIVE_TANK:  // move the robot in tank mode
			pLeftMotor->Set(localMessage.params.tankDrive.left);
			pRightMotor->Set(localMessage.params.tankDrive.right);
			break;

		case COMMAND_DRIVETRAIN_STOP:  // stop the robot
			pLeftMotor->Set(0.0);
			pRightMotor->Set(0.0);
			break;

		case COMMAND_SYSTEM_MSGTIMEOUT:  // what should we do if we do not get a timely message?
			break;

		default:
			break;
	}
}



