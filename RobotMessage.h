/** \file
 *  Messages used for intertask communications
 */

/** Defines the messages we pass from task to task.
 *
 * The RobotMessage struct is a data structure used to pass information to the
 * robot's components. It is composed of a command that indicates the action to
 * be carried out and a union of params that contain additional data.
 */

#ifndef ROBOT_MESSAGE_H
#define ROBOT_MESSAGE_H

/**
 \msc
 arcgradient = 8;
 robot [label="Main\nRobot"],
 auto [label="Autonomous"],
 check [label="Check\nList"],
 drive [label="Drive\nTrain"],
 test [label="Component\nExample"];
 robot=>* [label="SYSTEM_MSGTIMEOUT"];
 robot=>* [label="SYSTEM_OK"];
 robot=>* [label="SYSTEM_ERROR"];
 robot=>* [label="STATE_DISABLED"];
 robot=>* [label="STATE_AUTONOMOUS"];
 robot=>* [label="STATE_TELEOPERATED"];
 robot=>* [label="STATE_TEST"];
 robot=>* [label="STATE_UNKNOWN"];
 robot=>auto [label="RUN"];
 robot=>check [label="RUN"];
 robot=>drive [label="STOP"];
 robot=>drive [label="DRIVE_TANK"];
 robot=>drive [label="DRIVE_ARCADE"];
 auto=>drive [label="DRIVE_STRAIGHT"];
 auto=>drive [label="TURN"];
 drive=>auto [label="AUTONOMOUS_RESPONSE_OK"]
 drive=>auto [label="AUTONOMOUS_RESPONSE_ERROR"]

 robot=>test[label="TEST"];
 \endmsc

 */

enum MessageCommand {
	COMMAND_UNKNOWN,					//!< COMMAND_UNKNOWN
	COMMAND_SYSTEM_MSGTIMEOUT,			//!< COMMAND_SYSTEM_MSGTIMEOUT
	COMMAND_SYSTEM_OK,					//!< COMMAND_SYSTEM_OK
	COMMAND_SYSTEM_ERROR,				//!< COMMAND_SYSTEM_ERROR
	COMMAND_SYSTEM_CONSTANTS,

	COMMAND_ROBOT_STATE_DISABLED,		//!< Tells all components that the robot is disabled
	COMMAND_ROBOT_STATE_AUTONOMOUS,		//!< Tells all components that the robot is in auto
	COMMAND_ROBOT_STATE_TELEOPERATED,	//!< Tells all components that the robot is in teleop
	COMMAND_ROBOT_STATE_TEST,			//!< Tells all components that the robot is in test
	COMMAND_ROBOT_STATE_UNKNOWN,		//!< Tells all components that the robot's state is unknown

	COMMAND_AUTONOMOUS_RUN,				//!< Tells Autonomous to run
	COMMAND_AUTONOMOUS_COMPLETE,		//!< Tells all components that Autonomous is done running the script
	COMMAND_AUTONOMOUS_RESPONSE_OK,		//!< Tells Autonomous that a command finished running successfully
	COMMAND_AUTONOMOUS_RESPONSE_ERROR,	//!< Tells Autonomous that a command had a error while running
	COMMAND_CHECKLIST_RUN,				//!< Tells CheckList to run

	COMMAND_DRIVETRAIN_STOP,			//!< Tells Drivetrain to stop moving
	COMMAND_DRIVETRAIN_DRIVE_TANK,		//!< Tells Drivetrain to use tank drive
	COMMAND_DRIVETRAIN_DRIVE_CHEEZY,    //!< Tells Drivetrain to use cheesy drive
	COMMAND_DRIVETRAIN_AUTO_MOVE,    	//!< Tells Drivetrain to start moving
	COMMAND_DRIVETRAIN_AUTO_MMOVE,    	//!< Tells Drivetrain to move a measured distance
	COMMAND_DRIVETRAIN_AUTO_PMOVE,    	//!< Tells Drivetrain to move into proximity of barrier
	COMMAND_DRIVETRAIN_AUTO_TMOVE,    	//!< Tells Drivetrain to move for an amount of time
	COMMAND_DRIVETRAIN_TURN,
	COMMAND_DRIVETRAIN_PLED_ON,
	COMMAND_DRIVETRAIN_PLED_OFF,

	COMMAND_HOPPER_UP,
	COMMAND_HOPPER_DOWN,
	COMMAND_HOPPER_STOP,

	COMMAND_CLIMBER_UP,
	COMMAND_CLIMBER_DOWN,
	COMMAND_CLIMBER_STOP,
	COMMAND_AUTO_CLIMBER,

	COMMAND_GEARINTAKE_HOLD,
	COMMAND_GEARINTAKE_RELEASE,
	COMMAND_GEARINTAKE_TENSION,
	COMMAND_GEARINTAKE_STOP,


	//add new component messages here

	COMMAND_COMPONENT_TEST,				//!< COMMAND_COMPONENT_TEST
	COMMAND_LAST                        //!< COMMAND_LAST
};

struct MoveParams {
	float fLeft;
	float fRight;
};

struct MeasuredMoveParams {
	float fSpeed;
	float fDistance;
	float fTime;
};

struct ProximityMoveParams {
	float fSpeed;
	float fDistance;
	float fTime;
};

struct TimedMoveParams {
	float fSpeed;
	float fTime;
};

struct TurnParams {
	float fAngle;
	float fTimeout;
};

///Used to deliver joystick readings to Drivetrain
struct TankDriveParams {
	float left;
	float right;
};

///Used to deliver joystick readings to Drivetrain
struct CheezyDriveParams {
	float wheel;
	float throttle;
	bool bQuickturn;
};

struct ClimberParams {
	float ClimbUp;
	float ClimbDown;
};

struct HopperParams {
	float HopUp;
	float HopDown;
};

struct GearIntakeParams {
	float GearHold;
	float GearRelease;
};

struct SystemParams {
	float fBattery;
};


///Used to deliver autonomous values to Drivetrain
struct AutonomousParams {
	unsigned uMode;
	unsigned uDelay;
	///how long a function can run, maximum
	float timeout;
	///how long until a function performs
	float timein;

	///used by drivetrain for straight driving
	float driveSpeed;
	float driveDistance;
	float turnAngle;
	float driveTime;
};

///Contains all the parameter structures contained in a message
union MessageParams {
	TankDriveParams tankDrive;
	CheezyDriveParams cheezyDrive;
	MoveParams move;
	MeasuredMoveParams mmove;
	ProximityMoveParams pmove;
	TimedMoveParams tmove;
	TurnParams turn;
	ClimberParams climber;
	HopperParams hopper;
	GearIntakeParams gear;
	SystemParams system;
};

///A structure containing a command, a set of parameters, and a reply id, sent between components
struct RobotMessage {
	MessageCommand command;
	const char* replyQ;
	MessageParams params;
};

#endif //ROBOT_MESSAGE_H
