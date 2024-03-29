/** \file
 * Example of subsystem task declaration.
 *
 * This class calls the cheesy drive libraries
 *
 */

#ifndef CHEESYDRIVE_H
#define CHEESYDRIVE_H

/**
	A template class for creating new CHEESYDRIVEs
 */
#include <pthread.h>
#include <string>

//Robot
#include <WPILib.h>
#include "RobotParams.h"

// Structures to carry information about the drivetrain - must match cheezy code

struct DrivetrainGoal {
    double steering;
    double throttle;
    bool highgear;
    bool quickturn;
    bool control_loop_driving;
    double left_goal;
    double left_velocity_goal;
    double right_goal;
    double right_velocity_goal;
  };

struct DrivetrainPosition {
    double left_encoder;
    double right_encoder;
    double gyro_angle;
    double gyro_velocity;
    double battery_voltage;
    bool left_shifter_position;
    bool right_shifter_position;
  };

struct DrivetrainOutput {
    double left_voltage;
    double right_voltage;
    bool left_high;
    bool right_high;
  };

struct DrivetrainStatus {
    double robot_speed;
    double filtered_left_position;
    double filtered_right_position;
    double filtered_left_velocity;
    double filtered_right_velocity;

    double uncapped_left_voltage;
    double uncapped_right_voltage;
    bool output_was_capped;
  };

class CheesyLoop {

 public:
	CheesyLoop();
 	~CheesyLoop();

 	bool bEnableServo;

	static void *StartTask(void *pThis, const char* szComponentName, int iPriority)
	{
		pthread_setname_np(pthread_self(), szComponentName);
		pthread_setschedprio(pthread_self(), iPriority);
		((CheesyLoop *)pThis)->Run();
		return(NULL);
	}

	void Run(void);

 	void Update(const DrivetrainGoal &goal,
 	    const DrivetrainPosition &position,
 	    DrivetrainOutput &output,
 	    DrivetrainStatus &status,
 		bool bEnabled);
 private:
 	struct DrivetrainGoal currentGoal;
 	struct DrivetrainPosition currentPosition;
 	struct DrivetrainOutput currentOutput;
 	struct DrivetrainStatus currentStatus;
 	std::thread* pTask;

 	mutable priority_recursive_mutex mutexData;
 	bool bOutputEnabled;

 	void Iterate(const DrivetrainGoal *goal,
 				 const DrivetrainPosition *position,
 		         DrivetrainOutput *output,
 		         DrivetrainStatus *status);
 };

extern "C" void CheezyInit1296(void);

extern "C" void CheezyIterate1296(
    const DrivetrainGoal *goal,
    const DrivetrainPosition *position,
    DrivetrainOutput *output,
    DrivetrainStatus *status);



#endif			//CHEESYDRIVE_H
