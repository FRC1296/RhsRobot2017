/** \file
 * Class for our autonomous behaviours
 *
 *  This file contains our autonomous algorithms.  It should detect if we are in
 *  autonomous mode or not, select an algorithm based upon switch settings at
 *  the driver station and implement the behaviours till autonomous mode ends.
 */

#include <Autonomous.h>
#include <AutoParser.h>
#include <ComponentBase.h>
#include "WPILib.h"
#include <string.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

//Robot
#include <RobotParams.h>

using namespace std;

extern "C" {
}

bool Autonomous::CommandResponse(const char *szQueueName) {
	int iPipeXmt;
	bool bReturn = true;

	iPipeXmt = open(szQueueName, O_WRONLY);
	wpi_assert(iPipeXmt > 0);

	bReceivedCommandResponse = false;
	Message.replyQ = AUTONOMOUS_QUEUE;
	write(iPipeXmt, (char*) &Message, sizeof(RobotMessage));
	close(iPipeXmt);

	while (!bReceivedCommandResponse)
	{
		//purposefully empty
	}

	if(iAutoDebugMode)
	{
		printf("%0.3lf Response received\n", pDebugTimer->Get());
	}

	if (ReceivedCommand == COMMAND_AUTONOMOUS_RESPONSE_OK)
	{
		SmartDashboard::PutString("Auto Status","auto ok");
		bReturn = true;
	}
	else if (ReceivedCommand == COMMAND_AUTONOMOUS_RESPONSE_ERROR)
	{
		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		PRINTAUTOERROR;
		bReturn = false;
	}

	return bReturn;
}


//UNTESTED
//USAGE: MultiCommandResponse({DRIVETRAIN_QUEUE, CONVEYOR_QUEUE}, {COMMAND_DRIVETRAIN_STRAIGHT, COMMAND_CONVEYOR_SEEK_TOTE});
bool Autonomous::MultiCommandResponse(vector<char*> szQueueNames, vector<MessageCommand> commands) {
	//wait for several commands at once
	//check that queue list is as long as command list
	if(szQueueNames.size() != commands.size())
	{
		SmartDashboard::PutString("Auto Status","MULTICOMMAND error!");
		return false;
	}
	bool bReturn = true;
	int iPipeXmt;
	uResponseCount = 0;
	//vector<int> iPipesXmt = new vector<int>();

	bReceivedCommandResponse = false;

	//send messages to each component
	for (unsigned int i = 0; i < szQueueNames.size(); i++)
	{
		iPipeXmt = open(szQueueNames[i], O_WRONLY);
		wpi_assert(iPipeXmt > 0);

		Message.replyQ = AUTONOMOUS_QUEUE;
		Message.command = commands[i];
		write(iPipeXmt, (char*) &Message, sizeof(RobotMessage));
		close(iPipeXmt);
	}

	while (uResponseCount < szQueueNames.size())
	{
		while (!bReceivedCommandResponse)
		{
			//purposefully empty
		}

		if(iAutoDebugMode)
		{
			printf("%0.3lf Response received\n", pDebugTimer->Get());
		}

		if (ReceivedCommand == COMMAND_AUTONOMOUS_RESPONSE_OK)
		{
			SmartDashboard::PutString("Auto Status", "auto ok");
			bReturn = true;
		}
		else if (ReceivedCommand == COMMAND_AUTONOMOUS_RESPONSE_ERROR)
		{
			SmartDashboard::PutString("Auto Status", "EARLY DEATH!");
			bReturn = false;
		}
	}
	return bReturn;
}

bool Autonomous::CommandNoResponse(const char *szQueueName) {
	int iPipeXmt;

	iPipeXmt = open(szQueueName, O_WRONLY);
	wpi_assert(iPipeXmt > 0);

	write(iPipeXmt, (char*) &Message, sizeof(RobotMessage));
	close(iPipeXmt);
	return (true);
}

void Autonomous::Delay(float delayTime)
{
	double fWait;


	for (fWait = 0.0; fWait < delayTime; fWait += 0.01)
	{
		// if we are paused break the delay into pieces
		while (bPauseAutoMode)
		{
			Wait(0.02);
		}

		Wait(0.01);
	}
}

bool Autonomous::Begin(char *pCurrLinePos)
{
	//tell all the components who may need to know that auto is beginning
	Message.command = COMMAND_AUTONOMOUS_RUN;
	return (CommandNoResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::End(char *pCurrLinePos)
{
	//tell all the components who may need to know that auto is beginning
	Message.command = COMMAND_AUTONOMOUS_COMPLETE;
	CommandNoResponse(DRIVETRAIN_QUEUE);
	return (true);
}

bool Autonomous::Move(char *pCurrLinePos) {
	char *pToken;
	float fLeft;
	float fRight;

	// parse remainder of line to get length to move

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	fLeft = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	fRight = atof(pToken);

	if ((fabs(fLeft) > MAX_VELOCITY_PARAM)
			|| (fabs(fRight) > MAX_VELOCITY_PARAM))
	{
		return (false);
	}

	Message.command = COMMAND_DRIVETRAIN_AUTO_MOVE;
	Message.params.tankDrive.left =  -fLeft;
	Message.params.tankDrive.right = fRight;

	return (CommandNoResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::MeasuredMove(char *pCurrLinePos) {

	char *pToken;
	float fDistance;
	float fSpeed;
	float fTime;

	// parse remainder of line to get length to move

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		return (false);
	}

	fSpeed = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		return (false);
	}

	fDistance = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		return (false);
	}

	fTime = atof(pToken);

	// send the message to the drive train

	Message.command = COMMAND_DRIVETRAIN_AUTO_MMOVE;
	Message.params.mmove.fSpeed = fSpeed;
	Message.params.mmove.fDistance = fDistance;
	Message.params.mmove.fTime = fTime;

	return (CommandResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::MeasuredMoveProximity(char *pCurrLinePos) {

	char *pToken;
	float fDistance;
	float fSpeed;
	float fTime;

	// parse remainder of line to get length to move

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		return (false);
	}

	fSpeed = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		return (false);
	}

	fDistance = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		return (false);
	}

	fTime = atof(pToken);

	// send the message to the drive train

	Message.command = COMMAND_DRIVETRAIN_AUTO_PMOVE;
	Message.params.pmove.fSpeed = fSpeed;
	Message.params.pmove.fDistance = fDistance;
	Message.params.pmove.fTime= fTime;

	return (CommandResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::TimedMove(char *pCurrLinePos) {
	char *pToken;
	float fSpeed;
	float fTime;

	// parse remainder of line to get length to move

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","DEATH BY PARAMS!");
		PRINTAUTOERROR;
		return (false);
	}

	fSpeed = atof(pToken);
	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","DEATH BY PARAMS!");
		PRINTAUTOERROR;
		return (false);
	}

	fTime = atof(pToken);

	// send the message to the drive train
	Message.command = COMMAND_DRIVETRAIN_AUTO_TMOVE;
	Message.params.tmove.fSpeed = fSpeed;
	Message.params.tmove.fTime = fTime;
	return (CommandNoResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::Turn(char *pCurrLinePos) {
	char *pToken;
	float fAngle;
	float fTimeout;
	// parse remainder of line to get target angle and timeout
	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","DEATH BY PARAMS!");
		return (false);
	}

	fAngle = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","DEATH BY PARAMS!");
		return (false);
	}

	fTimeout = atof(pToken);

	// send the message to the drive train
	Message.command = COMMAND_DRIVETRAIN_TURN;
	Message.params.turn.fAngle= fAngle;
	Message.params.turn.fTimeout = fTimeout;
	return (CommandResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::GearRelease()
{
	// move measure distance forward/backward

	Message.command = COMMAND_GEARINTAKE_RELEASE;
	CommandNoResponse(GEARINTAKE_QUEUE);
	return (true);
}

bool Autonomous::GearHold()
{
	// move measure distance forward/backward

	Message.command = COMMAND_GEARINTAKE_HOLD;
	CommandNoResponse(GEARINTAKE_QUEUE);
	return (true);
}

bool Autonomous::GearHangMacro()
{
	// run the gear hangar macro, should only take 750ms or so

	Message.command = COMMAND_MACRO_HANGGEAR;
	CommandNoResponse(GEARFLOORINTAKE_QUEUE);
	CommandNoResponse(DRIVETRAIN_QUEUE);
	Wait(1.0);
	return (true);
}

bool Autonomous::Climber()
{
	Message.command = COMMAND_AUTO_CLIMBER;
	Message.params.climber.ClimbUp=1.0;
	return (CommandNoResponse(CLIMBER_QUEUE));
}

