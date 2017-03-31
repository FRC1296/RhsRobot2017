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
#include "PixyCam.h"
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

		// move to a point the requested distance away from the object

		fLastOffset = 1.0 - pPixiImagePosition->GetVoltage()/3.3*2.0;
		fStraightDriveDistance = pUltrasonic->GetRangeInches()/12.0 - distance;
		printf("fStraightDriveDistance in feet %f and counts %d \n", fStraightDriveDistance,
				(int)(fStraightDriveDistance/REVSPERFOOT*TALON_COUNTSPERREV));
		fStraightDriveDistance = fStraightDriveDistance/REVSPERFOOT*TALON_COUNTSPERREV;
	}
	else
	{
		fStraightDriveDistance = 0.0;
	}

	// attempt to adjust for stop time

	//if(fStraightDriveDistance > 0.0)
	//{
	//	fStraightDriveDistance -= (speed / FULLSPEED_FROMTALONS * 0.25);
	//}
	//else if(fStraightDriveDistance < 0.0)
	//{
	//	fStraightDriveDistance += (speed / FULLSPEED_FROMTALONS * 0.25);
	//}
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
				//printf("not auto or timed out \n");
				break;
			}
		}
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

	SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
}

void Drivetrain::StraightDriveLoop(float speed)
{
	float offset = 0.0;

	if(bMeasuredMove)
	{
		offset = (pGyro->GetAngle()-fTurnAngle)/30.0;
	}
	else if(bMeasuredMoveProximity)
	{
		if(pPixiImageDetect->Get())
		{
			// from Mittens code

			offset = 1.0 - pPixiImagePosition->GetVoltage()/3.3*2.0;

			if(fabs(fLastOffset - offset) > 0.25)
			{
				// too big a jump to be believable

				offset = fLastOffset;
			}
			else
			{
				fLastOffset = offset;
			}
		}

		//if(!pPixy->GetCentroid(offset))
		//{
		// nothing found, just drive straight
		//
		//            offset = (pGyro->GetAngle()-fTurnAngle)/45;
		//        }

		//if(fabs(offset) < 0.001)
		//{
			// nothing found, just drive straight

			//offset = (pGyro->GetAngle()-fTurnAngle)/30.0 ;
		//}
	}

	pLeftMotor->Set(-(speed - offset) * FULLSPEED_FROMTALONS* fBatteryVoltage / 12.0);
	pRightMotor->Set((speed + offset) * FULLSPEED_FROMTALONS* fBatteryVoltage / 12.0);
}


void Drivetrain::StartTurn(float angle, float time)
{
	pAutoTimer->Reset();
	pAutoTimer->Start();

	fTurnAngle = angle;
	fTurnTime = time;
	pGyro->Zero();
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
				pLeftMotor->Set(fNextMotor * FULLSPEED_FROMTALONS * fBatteryVoltage / 12.0);
				pRightMotor->Set(fNextMotor * FULLSPEED_FROMTALONS * fBatteryVoltage / 12.0);
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




