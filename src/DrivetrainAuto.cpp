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


void Drivetrain::StartStraightDrive (float speed, float distance, float time)
{
	// start a timer so we do not spend forever in this loop

	pAutoTimer->Reset();
	pAutoTimer->Start();

	// set relative encoder position, we'll measure from zero

	while(pRightMotor->GetEncPosition() && bInAuto)
	{
		pRightMotor->SetEncPosition(0);
		Wait(.005);
	}

	fTurnAngle = 0.0;
	pGyro->Zero();

	// remember the speed and time starting point

	fStraightDriveSpeed = speed;
	fStraightDriveTime = time;

	// estimate the total distance

	if(bMeasuredMove)
	{
		// move the requested distance
		fStraightDriveDistance = distance/REVSPERFOOT*TALON_COUNTSPERREV;
	}
	else if(bMeasuredMoveProximity)
	{
		pLed->Set(Relay::kForward);
		Wait(0.5);
		printf("Noah 2 %f %f \n", distance, pUltrasonic->GetRangeInches()/12.0);
		// move to a point the requested distance away from the object
		fStraightDriveDistance = pUltrasonic->GetRangeInches()/12.0 - distance;
		fStraightDriveDistance = fStraightDriveDistance/REVSPERFOOT*TALON_COUNTSPERREV;
	}
	else
	{
		fStraightDriveDistance = 0.0;
	}

	// attempt to adjust for stop time

	if(fStraightDriveDistance > 0.0)
	{
		fStraightDriveDistance -= (speed * TALON_COUNTSPERREV);
	}
	else if(fStraightDriveDistance < 0.0)
	{
		fStraightDriveDistance += (speed * TALON_COUNTSPERREV);
	}
}

void Drivetrain::IterateStraightDrive(void)
{
	if(bMeasuredMove || bMeasuredMoveProximity)
	{
		while(true)
		{
			if ((pAutoTimer->Get() < fStraightDriveTime) && bInAuto)
			{
				//SmartDashboard::PutNumber("travelenc", pRightOneMotor->GetEncPosition());
				//SmartDashboard::PutNumber("distenc", fStraightDriveDistance * (TALON_COUNTSPERREV * REVSPERFOOT));
				//SmartDashboard::PutNumber("velocity Right", pRightOneMotor->GetSpeed());
				//SmartDashboard::PutNumber("velocity Left", pLeftOneMotor->GetSpeed());

				if((float)abs(pRightMotor->GetEncPosition()) < fabs(fStraightDriveDistance))
				{
					StraightDriveLoop(fStraightDriveSpeed);
					Wait(.005);
				}
				else
				{
					printf("reached limit traveled %d , needed %d (%d) \n", pRightMotor->GetEncPosition(),
							(int)(fStraightDriveDistance),
							(int)(fStraightDriveDistance * (TALON_COUNTSPERREV * REVSPERFOOT)));

					break;
				}
			}
			else
			{
				printf("not auto or timed out \n");
				break;
			}
		}


 		// feed cheezy filters but do not activate motors
		RunCheezyDrive(false, 0.0, 0.0, false);

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
			RunCheezyDrive(false, 0.0, 0.0, false);
		}
	}

	bDrivingStraight = false;
	bMeasuredMoveProximity = false;
	pLeftMotor->Set(0.0);
	pRightMotor->Set(0.0);
	pLeftMotor->ClearError();
	pRightMotor->ClearError();
	pLeftMotor->StopMotor();
	pRightMotor->StopMotor();
	pLed->Set(Relay::kReverse);

		// feed cheezy filters but do not activate motors
	RunCheezyDrive(false, 0.0, 0.0, false);
	SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
}

void Drivetrain::StraightDriveLoop(float speed)
{
	float offset;

	//if(bMeasuredMove)
	{
		offset = (pGyro->GetAngle()-fTurnAngle)/45;
		pLeftMotor->Set(-(speed - offset) * FULLSPEED_FROMTALONS);
		pRightMotor->Set((speed + offset) * FULLSPEED_FROMTALONS);
	}
	/*else if(bMeasuredMoveProximity)
	{
		//insert code to get angle offset from pixi

		if(pPixiImageDetect->Get())
		{
			// from Mittens code

			offset = 1.0 - pPixiImagePosition->GetVoltage()/3.3*2.0;
		}
		else
		{
			offset = 0.0;
		}

		pLeftMotor->Set(-(speed - offset) * FULLSPEED_FROMTALONS);
		pRightMotor->Set((speed + offset) * FULLSPEED_FROMTALONS);
	}*/

    // feed cheezy filters but do not activate motors
	RunCheezyDrive(false, -(pGyro->GetAngle()-fTurnAngle)/45, speed, false);
}


void Drivetrain::StartTurn(float angle, float time)
{
	pAutoTimer->Reset();
	pAutoTimer->Start();

	fTurnAngle = angle;
	fTurnTime = time;
	pGyro->Zero();

	printf("starting to turn %f %f\n", angle, time);
}

void Drivetrain::IterateTurn(void)
{
	float fCurrentAngle;
	float fCurrentError;
	float fNextMotor;

	if(bTurning)
	{
		while(true)
		{
			fCurrentAngle = pGyro->GetAngle();
			fCurrentError = fCurrentAngle - fTurnAngle;
			fNextMotor = fCurrentError/180.0;

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
				pLeftMotor->Set(fNextMotor * FULLSPEED_FROMTALONS / 10.0);
				pRightMotor->Set(fNextMotor * FULLSPEED_FROMTALONS / 10.0);
			}
			else
			{
				break;
			}

			Wait(0.005);
		}

		pLeftMotor->Set(0.0);
		pRightMotor->Set(0.0);
		pLeftMotor->ClearError();
		pRightMotor->ClearError();
		pLeftMotor->StopMotor();
		pRightMotor->StopMotor();
		bTurning = false;
		SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
	}
}



