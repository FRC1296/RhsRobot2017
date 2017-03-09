/** \file
 * Implementation of class to drive the pallet jack.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control the robot's wheels.
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

#include "Drivetrain.h"
#include "CheesyDrive.h"
#include "RobotParams.h"


using namespace std;


void Drivetrain::StartStraightDrive (float speed, float time, float distance)
{
	// start a timer so we do not spend forever in this loop

	pAutoTimer->Reset();
	pAutoTimer->Start();

	// set relative encoder position, we'll measure from zero

	pLeftMotor->SetEncPosition(0);

	// set relative angle, we'll measure from current angle

	fTurnAngle = pGyro->GetAngle();

	// remember the speed and time starting point

	fStraightDriveSpeed = speed;
	fStraightDriveTime = time;

	// estimate the total distance

	fStraightDriveDistance = (distance/12)/(REVSPERFOOT/TALON_COUNTSPERREV)*4;
}

void Drivetrain::IterateStraightDrive(void)
{
	if(bMeasuredMove)
	{
		while(true)
		{
			if ((pAutoTimer->Get() < fStraightDriveTime) && bInAuto)
			{
				//SmartDashboard::PutNumber("travelenc", pRightOneMotor->GetEncPosition());
				//SmartDashboard::PutNumber("distenc", fStraightDriveDistance * (TALON_COUNTSPERREV * REVSPERFOOT));
				//SmartDashboard::PutNumber("velocity Right", pRightOneMotor->GetSpeed());
				//SmartDashboard::PutNumber("velocity Left", pLeftOneMotor->GetSpeed());

				if(pRightMotor->GetEncPosition() < fStraightDriveDistance
						&& pRightMotor->GetEncPosition() > -fStraightDriveDistance)
				{
					StraightDriveLoop(fStraightDriveSpeed);
					Wait(.005);
				}
				else
				{
					//printf("reached limit traveled %d , needed %d \n", pRightOneMotor->GetEncPosition(),
					//		(int)(fStraightDriveDistance * (TALON_COUNTSPERREV * REVSPERFOOT)));
					break;
				}
			}
			else
			{
				//printf("not auto or timed out \n");
				break;
			}
		}

		bDrivingStraight = false;
		bMeasuredMove = false;
		pLeftMotor->Set(0.0);
		pRightMotor->Set(0.0);
 		// feed cheezy filters but do not activate motors
		RunCheezyDrive(true, 0.0, 0.0, false);
		SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
	}
	else if(bMeasuredMoveProximity)
	{
		//printf("in move to ultrasonic distance \n");

		while(true)
		{
			if ((pAutoTimer->Get() < fStraightDriveTime) && bInAuto)
			{
// TODO check ultrasonic position
// distance here is from wall
// what should max distance be?  maybe a function of sensor function
// pUltrasonic->GetRangeInches()
				if((pRightMotor->GetEncPosition() < fMaxUltrasonicDistance)
						&& (pRightMotor->GetEncPosition() > -fMaxUltrasonicDistance) &&
						(pUltrasonic->GetRangeInches() < fStraightDriveDistance))
				{
					StraightDriveLoop(fStraightDriveSpeed);
					Wait(.005);
				}
				else
				{
					//printf("reached limit traveled %d , needed %d \n", pRightMotor->GetEncPosition(),
					//		(int)(fStraightDriveDistance * (TALON_COUNTSPERREV * REVSPERFOOT)));
					break;
				}
			}
			else
			{
				//printf("not auto or timed out \n");
				break;
			}
		}

		bDrivingStraight = false;
		bMeasuredMoveProximity = false;
		pLeftMotor->Set(0.0);
		pRightMotor->Set(0.0);

 		// feed cheezy filters but do not activate motors
		RunCheezyDrive(true, 0.0, 0.0, false);

		SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
	}
	else
	{
		if ((pAutoTimer->Get() < fStraightDriveTime) && bInAuto)
		{
			StraightDriveLoop(fStraightDriveSpeed);
		}
		else
		{
			bDrivingStraight = false;
			pLeftMotor->Set(0.0);
			pRightMotor->Set(0.0);

	 		// feed cheezy filters but do not activate motors
			RunCheezyDrive(true, 0.0, 0.0, false);
		}
	}
}

void Drivetrain::StraightDriveLoop(float speed)
{
	float offset = (pGyro->GetAngle()-fTurnAngle)/45;

	pLeftMotor->Set(-(speed - offset) * FULLSPEED_FROMTALONS);
	pRightMotor->Set((speed + offset) * FULLSPEED_FROMTALONS);

    // feed cheezy filters but do not activate motors
	RunCheezyDrive(true, -(pGyro->GetAngle()-fTurnAngle)/45, speed, false);
}


void Drivetrain::StartTurn(float angle, float time)
{
	float fCurrentAngle = pGyro->GetAngle();
	pAutoTimer->Reset();
	pAutoTimer->Start();

	// convert to +/- 180 degrees

	if(fCurrentAngle > 0.0){
		while(fCurrentAngle > 180.0){
			pGyro->SetAngle(fCurrentAngle - 360.0);
		}
	}else{
		while(fCurrentAngle < -180.0){
			pGyro->SetAngle(fCurrentAngle + 360.0);
		}
	}

	fTurnAngle = angle;
	fTurnTime = time;
}

void Drivetrain::IterateTurn(void)
{
	float fCurrentAngle = pGyro->GetAngle();
	float fCurrentError = fCurrentAngle - fTurnAngle;
	float fNextMotor = fCurrentError/180.0 * 1.0;

	if((fNextMotor > 0.0) && (fNextMotor < fMinimumTurnSpeed))
	{
		fNextMotor = fMinimumTurnSpeed;
	}
	else if((fNextMotor < 0.0) && (fNextMotor > -fMinimumTurnSpeed))
	{
		fNextMotor = -fMinimumTurnSpeed;
	}

	if(((fCurrentError >= 1.0) || (fCurrentError <= -1.0)) && (pAutoTimer->Get() < fTurnTime) && bInAuto)
	{
		if(fCurrentError > 0.0)
		{
			pLeftMotor->Set(0);
			pRightMotor->Set(fNextMotor * FULLSPEED_FROMTALONS);
		}
		else
		{
			pLeftMotor->Set(fNextMotor * FULLSPEED_FROMTALONS);
			pRightMotor->Set(0);
		}
	}
	else
	{
		pLeftMotor->Set(0.0);
		pRightMotor->Set(0.0);
		bTurning = false;
		SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
	}

}




