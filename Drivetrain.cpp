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

#include <iostream>
#include <algorithm>

#include "Drivetrain.h"
#include "CheesyDrive.h"
#include "PixyCam.h"
#include "RobotParams.h"


using namespace std;

Drivetrain::Drivetrain() :
		ComponentBase(DRIVETRAIN_TASKNAME, DRIVETRAIN_QUEUE,
				DRIVETRAIN_PRIORITY) {

	fBatteryVoltage = 12.0;

	// create all the objects used in this thread

	pLeftMotor = new CANTalon(CAN_DRIVETRAIN_LEFT_MOTOR);
	pRightMotor = new CANTalon(CAN_DRIVETRAIN_RIGHT_MOTOR);
	pLeftMotorSlave = new CANTalon(CAN_DRIVETRAIN_LEFT_MOTOR_SLAVE);
	pRightMotorSlave = new CANTalon(CAN_DRIVETRAIN_RIGHT_MOTOR_SLAVE);

	wpi_assert(pLeftMotor && pRightMotor);

	// setup for closed loop operation with VP encoders
	pLeftMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
	pLeftMotor->SelectProfileSlot(0);
	pLeftMotor->SetPID(TALON_PTERM_L, TALON_ITERM_L, TALON_DTERM_L, TALON_FTERM_L);		// PIDF
	pLeftMotor->SetIzone(TALON_IZONE);
	pLeftMotor->SetInverted(true);
	pLeftMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
	pLeftMotor->SetControlMode(CANTalon::kPercentVbus);

	pLeftMotorSlave->SetControlMode(CANSpeedController::kFollower);
	pLeftMotorSlave->Set(CAN_DRIVETRAIN_LEFT_MOTOR);

	// setup for closed loop operation with VP encoders

	pRightMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
	pRightMotor->SelectProfileSlot(0);
	pRightMotor->SetPID(TALON_PTERM_R, TALON_ITERM_R, TALON_DTERM_R, TALON_FTERM_R);  // PIDF
	pRightMotor->SetIzone(TALON_IZONE);
	pRightMotor->SetInverted(true);
	pRightMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
	pRightMotor->SetControlMode(CANTalon::kPercentVbus);

	pRightMotorSlave->SetControlMode(CANSpeedController::kFollower);
	pRightMotorSlave->Set(CAN_DRIVETRAIN_RIGHT_MOTOR);

	wpi_assert(pLeftMotor->IsAlive());
	wpi_assert(pRightMotor->IsAlive());
	wpi_assert(pLeftMotorSlave->IsAlive());
	wpi_assert(pRightMotorSlave->IsAlive());

	pUltrasonic = new Ultrasonic(DIO_ULTRASONIC_OUTPUT, DIO_ULTRASONIC_INPUT);
	pUltrasonic->SetAutomaticMode(true);

	pLed = new Relay(RELAY_LED);
	pPixiImageDetect = new DigitalInput(DIO_PIXI);
	pPixiImagePosition = new AnalogInput(AIO_PIXI);

	bUnderServoControl = false;
	bDrivingStraight = false;
	bTurning = false;
	bMeasuredMove = false;
	bMeasuredMoveProximity = false;
	bInAuto = false;

	fStraightDriveDistance = 0.0;

	pAutoTimer = new Timer();
	wpi_assert(pAutoTimer);
	pAutoTimer->Start();

	pRunTimer = new Timer();
	wpi_assert(pRunTimer);
	pRunTimer->Start();

	pGyro = new ADXRS453Z();
	wpi_assert(pGyro);

	fStraightDriveDistance = 0.0;
	fStraightDriveTime = 0.0;
	fStraightDriveSpeed = 0.0;
	fTurnAngle = 0.0;
	fTurnTime = 0.0;

	pCheezy = new CheesyLoop();
	pPixy = new PixyCam();

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
			pCheezy->bEnableServo = false;
			bUnderServoControl = true;
			bInAuto = true;
			pLeftMotor->SetControlMode(CANTalon::kSpeed);
			pRightMotor->SetControlMode(CANTalon::kSpeed);
			pLeftMotor->Set(0.0);
			pRightMotor->Set(0.0);
			pGyro->Zero();
			break;

		case COMMAND_ROBOT_STATE_TEST:
		case COMMAND_ROBOT_STATE_TELEOPERATED:
		case COMMAND_ROBOT_STATE_DISABLED:
		case COMMAND_ROBOT_STATE_UNKNOWN:
		default:
			pCheezy->bEnableServo = true;
			bUnderServoControl = false;
			bInAuto = false;
			pLeftMotor->SetControlMode(CANTalon::kPercentVbus);
			pRightMotor->SetControlMode(CANTalon::kPercentVbus);
			pLeftMotor->Set(0.0);
			pRightMotor->Set(0.0);
			break;
	}
}

void Drivetrain::Run() {

	switch(localMessage.command)
	{
	//Timing is set across this and gear intake side of macro
		case COMMAND_MACRO_HANGGEAR:
			Wait(0.200);

		    if(bUnderServoControl)
			{
				pLeftMotor->Set(0.33 * FULLSPEED_FROMTALONS);
				pRightMotor->Set(-0.33 * FULLSPEED_FROMTALONS);
				Wait(0.500);
				// make darn sure it stops !
				pLeftMotor->Set(0.0);
				pRightMotor->Set(0.0);
				pLeftMotor->ClearError();
				pRightMotor->ClearError();
				pLeftMotor->StopMotor();
				pRightMotor->StopMotor();
			}
			else
			{
				int loop_count = 0;

				while(loop_count<500)
				{
					RunCheezyDrive(true, 0.0, -0.33, false);
					loop_count += 50;
					Wait(.05);
				}
			}

			ClearMessages();
			break;

		case COMMAND_DRIVETRAIN_DRIVE_TANK:  // move the robot in tank mode
			pLeftMotor->Set(localMessage.params.tankDrive.left);
			pRightMotor->Set(localMessage.params.tankDrive.right);
			break;

		case COMMAND_DRIVETRAIN_STOP:  // stop the robot
			pLeftMotor->Set(0.0);
			pRightMotor->Set(0.0);
			break;

		case COMMAND_DRIVETRAIN_DRIVE_CHEEZY:
			RunCheezyDrive(true, localMessage.params.cheezyDrive.wheel,
					localMessage.params.cheezyDrive.throttle, localMessage.params.cheezyDrive.bQuickturn);
			break;

		case COMMAND_SYSTEM_CONSTANTS:
			fBatteryVoltage = localMessage.params.system.fBattery;
			break;

		case COMMAND_DRIVETRAIN_AUTO_MOVE:
			bDrivingStraight = false;
			bTurning = false;

		    if(bUnderServoControl)
			{
				pLeftMotor->Set(localMessage.params.move.fLeft * FULLSPEED_FROMTALONS);
				pRightMotor->Set(localMessage.params.move.fRight * FULLSPEED_FROMTALONS);
			}
			else
			{
				pLeftMotor->Set(localMessage.params.move.fLeft);
				pRightMotor->Set(localMessage.params.move.fRight);
			}
			break;

		case COMMAND_DRIVETRAIN_AUTO_MMOVE:
			bMeasuredMove = true;
			bMeasuredMoveProximity = false;
			bDrivingStraight = true;
			bTurning = false;

			StartStraightDrive(localMessage.params.mmove.fSpeed,
	 				localMessage.params.mmove.fDistance,
					localMessage.params.mmove.fTime);

	 		IterateStraightDrive();
			break;

		case COMMAND_DRIVETRAIN_AUTO_PMOVE:
			bMeasuredMove = false;
			bMeasuredMoveProximity = true;
			bDrivingStraight = true;
			bTurning = false;

			StartStraightDrive(localMessage.params.mmove.fSpeed,
	 				localMessage.params.mmove.fDistance, localMessage.params.mmove.fTime);

	 		IterateStraightDrive();
			break;

		case COMMAND_DRIVETRAIN_TURN:
			bDrivingStraight = false;
			bTurning = true;
			StartTurn(localMessage.params.turn.fAngle, localMessage.params.turn.fTimeout);

			IterateTurn();
			break;

		case COMMAND_DRIVETRAIN_PLED_ON:
			pLed->Set(Relay::kForward);
			printf("sup bro");
			break;

		case COMMAND_DRIVETRAIN_PLED_OFF:
			pLed->Set(Relay::kReverse);
			break;

		case COMMAND_SYSTEM_MSGTIMEOUT:  // what should we do if we do not get a timely message?
			break;

		default:
			break;
	}


	if(iLoop % 10 == 0)
	{
		//float fCentroid = 1.0 - pPixiImagePosition->GetVoltage()/3.3*2.0;
		float fCentroid = pPixiImagePosition->GetVoltage();
		int iRange = pUltrasonic->GetRangeInches();
		SmartDashboard::PutNumber("ultrasonic", iRange);

		SmartDashboard::PutNumber("Battery", fBatteryVoltage);
		SmartDashboard::PutNumber("angle", pGyro->GetAngle());
		SmartDashboard::PutNumber("left encoder", -pLeftMotor->GetEncPosition() * METERS_PER_COUNT);
		SmartDashboard::PutNumber("right encoder", pRightMotor->GetEncPosition() * METERS_PER_COUNT);

		if(pPixiImageDetect->Get())
		//if(pPixy->GetCentroid(fCentroid))
		{
			SmartDashboard::PutBoolean("Pixi Detect", true);
			SmartDashboard::PutNumber("Pixi Raw", fCentroid);
		}
		else
		{
			SmartDashboard::PutBoolean("Pixi Detect", false);
			//SmartDashboard::PutNumber("Pixi Raw", 999.9);
		}
	}
}

void Drivetrain::RunCheezyDrive(bool bEnabled, float fWheel, float fThrottle, bool bQuickturn)
{
    struct DrivetrainGoal Goal;
    struct DrivetrainPosition Position;
    struct DrivetrainOutput Output;
    struct DrivetrainStatus Status;



	if(bQuickturn)
	{
		Goal.steering = -pow(fWheel,3);
	}
	else
	{
		Goal.steering = -fWheel;
	}

    Goal.throttle = fThrottle;
    Goal.quickturn = bQuickturn;
    Goal.control_loop_driving = false;
    Goal.highgear = false;
    Goal.left_velocity_goal = 0.0;
    Goal.right_velocity_goal = 0.0;
    Goal.left_goal = 0.0;
    Goal.right_goal = 0.0;

    Position.left_encoder = -pLeftMotor->GetEncPosition() * METERS_PER_COUNT;
    Position.right_encoder = pRightMotor->GetEncPosition() * METERS_PER_COUNT;
    Position.gyro_angle = pGyro->GetAngle() * 3.141519 / 180.0;
    Position.gyro_velocity = pGyro->GetRate() * 3.141519 / 180.0;
    Position.battery_voltage = fBatteryVoltage;
    Position.left_shifter_position = true;
    Position.right_shifter_position = false;

    if(bEnabled)
    {
    	// if enabled and normal operation

    	pCheezy->Update(Goal, Position, Output, Status, true);
        pLeftMotor->Set(-Output.left_voltage / 12.0);
        pRightMotor->Set(Output.right_voltage / 12.0);
    }
    else
    {
        // if the robot is not running

    	pCheezy->Update(Goal, Position, Output, Status, false);
    }
}


