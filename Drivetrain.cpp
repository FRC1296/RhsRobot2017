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
#include <CANTalon.h>
#include <string>
#include <iostream>
#include <algorithm>
#include <cmath>
#include "Drivetrain.h"			//For the local header file
#include "RobotParams.h"


using namespace std;

Drivetrain::Drivetrain() :
		ComponentBase(DRIVETRAIN_TASKNAME, DRIVETRAIN_QUEUE,
				DRIVETRAIN_PRIORITY) {
	// create all the objects used in this thread

	pLeftMotor1 = new CANTalon(CAN_DRIVETRAIN_LEFT_MOTOR1);
	pLeftMotor2 = new CANTalon(CAN_DRIVETRAIN_LEFT_MOTOR2);
	pRightMotor1 = new CANTalon(CAN_DRIVETRAIN_RIGHT_MOTOR1);
	pRightMotor2 = new CANTalon(CAN_DRIVETRAIN_RIGHT_MOTOR2);

	pLeftMotor1->SetVoltageRampRate(48.0);
	pRightMotor1->SetVoltageRampRate(48.0);

	wpi_assert(pLeftMotor1 && pLeftMotor2 && pRightMotor1 && pRightMotor2);

	// initialize the motor on the left side

	pLeftMotor1->ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
	pLeftMotor1->SetControlMode(CANTalon::kPercentVbus);
	pLeftMotor1->SetFeedbackDevice(CANTalon::QuadEncoder);

	pLeftMotor2->SetControlMode(CANTalon::kFollower);
	pLeftMotor2->Set(CAN_DRIVETRAIN_LEFT_MOTOR1);

	// initialize the motor on the right side

	pRightMotor1->ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
	pRightMotor1->SetControlMode(CANTalon::kPercentVbus);
	pRightMotor1->SetFeedbackDevice(CANTalon::QuadEncoder);

	pRightMotor2->SetControlMode(CANTalon::kFollower);
	pRightMotor2->Set(CAN_DRIVETRAIN_RIGHT_MOTOR1);

	pGyro = new ADXRS453Z();
	wpi_assert(pGyro);
	pGyro->Zero();

	pTask = new std::thread(&Drivetrain::StartTask, this,
			DRIVETRAIN_TASKNAME, DRIVETRAIN_PRIORITY);
	wpi_assert(pTask);
}

Drivetrain::~Drivetrain()			//Destructor
{
	// clean up here, delete things in reverse order

	delete pTask;
	delete pLeftMotor1;
	delete pRightMotor1;
	delete pLeftMotor2;
	delete pRightMotor2;
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
double speedleft;
double speedright;
double J1;
double J2;
double leftVelocity;
double rightVelocity;
int leftPosition;
int rightPosition;
float angle;

	switch(localMessage.command)
	{
		case COMMAND_DRIVETRAIN_DRIVE_TANK:  // move the robot in tank mode
			speedleft = tan((3.14159267/4)*(localMessage.params.tankDrive.left));
			speedright = tan((3.14159267/4)*(localMessage.params.tankDrive.right));
			pLeftMotor1->Set(speedleft*-1);
			//pLeftMotor2->Set(speedleft*-1);
			pRightMotor1->Set(speedright*-1);
			//pRightMotor2->Set(speedright*-1);
			SmartDashboard::PutNumber("tank LEFT", speedleft);
			SmartDashboard::PutNumber("tank RIGHT", speedright);

			SmartDashboard::PutNumber("L1 (1)", pLeftMotor1->GetOutputCurrent());
			SmartDashboard::PutNumber("L2 (2)", pLeftMotor2->GetOutputCurrent());
			SmartDashboard::PutNumber("R1 (3)", pRightMotor1->GetOutputCurrent());
			SmartDashboard::PutNumber("R2 (4)", pRightMotor2->GetOutputCurrent());



			break;

		case COMMAND_DRIVETRAIN_DRIVE_ARCADE:
			J1 = localMessage.params.arcadeDrive.vertical;
			J2 = localMessage.params.arcadeDrive.horizontal;
			speedleft =  (J1 - J2);
			speedright = (J1 + J2);
			SmartDashboard::PutNumber("LEFT", speedleft*-1);
			SmartDashboard::PutNumber("HORIZONTAL", J2);
			SmartDashboard::PutNumber("RIGHT", speedright);

			leftVelocity=pLeftMotor1->GetEncVel();
			rightVelocity=pRightMotor1->GetEncVel();
			leftPosition=pLeftMotor1->GetEncPosition();
			rightPosition=pRightMotor1->GetEncPosition();
			SmartDashboard::PutNumber("LEFT VELOCITY", leftVelocity);
			SmartDashboard::PutNumber("RIGHT VELOCITY", rightVelocity);
			SmartDashboard::PutNumber("LEFT POSITION", leftPosition);
			SmartDashboard::PutNumber("RIGHT POSITION", rightPosition);

			angle = pGyro->GetAngle();
			SmartDashboard::PutNumber("GYRO ANGLE", angle);

			pLeftMotor1->Set(speedleft*-1);
			//pLeftMotor2->Set(speedleft*-1);
			pRightMotor1->Set(speedright);
			//pRightMotor2->Set(speedright*-1);

			break;

		case COMMAND_DRIVETRAIN_STOP:
			pLeftMotor1->Set(0);
			//pLeftMotor2->Set(0);
			pRightMotor1->Set(0);
			//pRightMotor2->Set(0);
			break;

		case COMMAND_SYSTEM_MSGTIMEOUT:  // what should we do if we do not get a timely message?
			break;

		default:
			break;
	}
}



