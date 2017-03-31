/** \file
 * Definitions of class to control the drive train.
 *
 * This classes is derived from the standard Component base class and includes
 * definitions for the devices used to control the pallet jack wheels.
 */

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

//Robot
#include <WPILib.h>
#include <CANTaLon.h>

#include <ComponentBase.h>			//For ComponentBase class
#include <pthread.h>

#include "ADXRS453Z.h"

/*
Selected Device:0:Quad Encoder
Pos (rot):   1.360 	Velocity (RPM):   0.00
Pos:2786 	Velocity:0

Quad Encoder (4x)
Pos (rot):   1.360 	Velocity (RPM):   0.00
Pos:2786 	Velocity:0
Pins: A=1 B=1 Idx=1
Idx rise edges:0

... 8 feet later

Selected Device:0:Quad Encoder
Pos (rot):   -0.480 	Velocity (RPM):   0.00
Pos:-986 	Velocity:0

Quad Encoder (4x)
Pos (rot):   -0.480 	Velocity (RPM):   0.00
Pos:-986 	Velocity:0
Pins: A=1 B=1 Idx=1
Idx rise edges:0

so ...

1.36+.480  rotations = 1.84 rotations  ?
2786 + 986 counts = 3772 = 7+ revolutions = 7.56 feet
2 rev =


*/

// constants used to tune TALONS

// measured 546 left and 534 right on the ground,
// 641 left and 562 right on blocks on 21 March 2016

const float FULLSPEED_FROMTALONS = 	300.0;	// measured on the robot in RPMs
const float TALON_FTERM_L = 		1.364;	// From CTRE manual, section 12.4
const float TALON_PTERM_L = 		(0.372);
const float TALON_ITERM_L = 		(TALON_PTERM_L / 100.0);
const float TALON_DTERM_L = 		(TALON_PTERM_L * 10.0);
const float TALON_FTERM_R = 		(TALON_FTERM_L * 0.75);	// right side always seems stronger
const float TALON_PTERM_R = 		(TALON_PTERM_L * 0.75);
const float TALON_ITERM_R = 		(TALON_PTERM_R / 100.0);
const float TALON_DTERM_R = 		(TALON_PTERM_R * 10.0);
const float TALON_MAXRAMP =			60;		// 200ms
const float TALON_IZONE	=			128;
const float TALON_COUNTSPERREV =	512;	// from CTRE docs
const float REVSPERFOOT = (3.14159 * 2.0 * 2.0 / 12.0);
const double METERS_PER_COUNT = (REVSPERFOOT / (double)TALON_COUNTSPERREV);

const float fMinimumTurnSpeed = 0.20;
const float fMaxUltrasonicDistance = (25.0/REVSPERFOOT*TALON_COUNTSPERREV);  //25 feet

const int iIdealGearDistance = 10;    // 10" need to get this right (used for indicator on panel)
const int iIdealGearDistanceError = 1;

class CheesyLoop;
class PixyCam;

class Drivetrain : public ComponentBase
{
public:
	Drivetrain();
	~Drivetrain();

	static void *StartTask(void *pThis, const char* szComponentName, int iPriority)
	{
		pthread_setname_np(pthread_self(), szComponentName);
		pthread_setschedprio(pthread_self(), iPriority);
		((Drivetrain *)pThis)->DoWork();
		return(NULL);
	}

private:
	void OnStateChange();
	void Run();
	void RunCheezyDrive(bool, float, float, bool);
	void StartStraightDrive (float, float, float);
	void IterateStraightDrive(void);
	void StraightDriveLoop(float);
	void StartTurn(float, float);
	void IterateTurn(void);

	CANTalon* pLeftMotor;
	CANTalon* pRightMotor;
	CANTalon* pLeftMotorSlave;
	CANTalon* pRightMotorSlave;
	Timer *pAutoTimer;
	Timer* pRunTimer;
	Ultrasonic *pUltrasonic;
	Relay *pLed;
	DigitalInput *pPixiImageDetect;
	AnalogInput *pPixiImagePosition;

	ADXRS453Z *pGyro;
	float fBatteryVoltage;
	float fStraightDriveSpeed;
	float fStraightDriveTime;
	float fStraightDriveDistance;
	float fTurnAngle;
	float fTurnTime;
	float fLastOffset;
	bool bUnderServoControl;
	bool bMeasuredMove;
	bool bMeasuredMoveProximity;
	bool bDrivingStraight;
	bool bTurning;
	bool bInAuto;

	CheesyLoop *pCheezy;
	PixyCam *pPixy;
};

#endif			//DRIVETRAIN_H
