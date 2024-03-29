 /** \file
 *  Defines task parameters, hardware assignments and controller button/axis assignment.
 *
 * This header contains basic parameters for the robot. All parameters must be constants with internal
 * linkage, otherwise the One Definition Rule will be violated.
 */

// TODO: please go over these items with a knowledgeable mentor and check to see what we need/don't need
#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H

//Robot
#include <JoystickLayouts.h>			//For joystick layouts

//Robot Params
const char* const ROBOT_NAME =		"RhsRobot2017";	//Formal name
const char* const ROBOT_NICKNAME =  "Unknown";		//Nickname
const char* const ROBOT_VERSION =	"0.5";			//Version

//Utility Functions - Define commonly used operations here
#define ABLIMIT(a,b)		if(a > b) a = b; else if(a < -b) a = -b;
#define TRUNC_THOU(a)		((int)(1000 * a)) * .001
#define TRUNC_HUND(a)		((int)(100 * a)) * .01
#define PRINTAUTOERROR		printf("Early Death! %s %i \n", __FILE__, __LINE__);

//Task Params - Defines component task priorites relative to the default priority.
//EXAMPLE: const int DRIVETRAIN_PRIORITY = DEFAULT_PRIORITY -2;
const int DEFAULT_PRIORITY      = 50;
const int COMPONENT_PRIORITY 	= DEFAULT_PRIORITY;
const int DRIVETRAIN_PRIORITY 	= DEFAULT_PRIORITY;
const int CHEESY_PRIORITY 	    = DEFAULT_PRIORITY;
const int PIXI_PRIORITY 	    = DEFAULT_PRIORITY;
const int AUTONOMOUS_PRIORITY 	= DEFAULT_PRIORITY;
const int AUTOEXEC_PRIORITY 	= DEFAULT_PRIORITY;
const int AUTOPARSER_PRIORITY 	= DEFAULT_PRIORITY;
const int CLIMBER_PRIORITY		= DEFAULT_PRIORITY;
const int HOPPER_PRIORITY		= DEFAULT_PRIORITY;
const int GEARINTAKE_PRIORITY	= DEFAULT_PRIORITY;
const int GEARFLOORINTAKE_PRIORITY	= DEFAULT_PRIORITY;

//Task Names - Used when you view the task list but used by the operating system
//EXAMPLE: const char* DRIVETRAIN_TASKNAME = "tDrive";
const char* const COMPONENT_TASKNAME	= "tComponent";
const char* const DRIVETRAIN_TASKNAME	= "tDrive";
const char* const CHEESY_TASKNAME	    = "tCheesy";
const char* const PIXI_TASKNAME	    	= "tPixi";
const char* const AUTONOMOUS_TASKNAME	= "tAuto";
const char* const AUTOEXEC_TASKNAME		= "tAutoEx";
const char* const AUTOPARSER_TASKNAME	= "tParse";
const char* const CLIMBER_TASKNAME		= "tClimber";
const char* const HOPPER_TASKNAME		= "tHopper";
const char* const GEARINTAKE_TASKNAME	= "tGearIntake";
const char* const GEARFLOORINTAKE_TASKNAME	= "tGearFloor";

//TODO change these variables throughout the code to PIPE or whatever instead  of QUEUE
//Queue Names - Used when you want to open the message queue for any task
//NOTE: 2015 - we use pipes instead of queues
//EXAMPLE: const char* DRIVETRAIN_TASKNAME = "tDrive";

const char* const COMPONENT_QUEUE 	= "/tmp/qComp";
const char* const DRIVETRAIN_QUEUE 	= "/tmp/qDrive";
const char* const AUTONOMOUS_QUEUE 	= "/tmp/qAuto";
const char* const AUTOPARSER_QUEUE 	= "/tmp/qParse";
const char* const CLIMBER_QUEUE		= "/tmp/qClimber";
const char* const HOPPER_QUEUE		= "/tmp/qHopper";
const char* const GEARINTAKE_QUEUE	= "/tmp/qGearIntake";
const char* const GEARFLOORINTAKE_QUEUE	= "/tmp/qGearFloor";

//PWM Channels - Assigns names to PWM ports 1-10 on the Roborio
//EXAMPLE: const int PWM_DRIVETRAIN_FRONT_LEFT_MOTOR = 1;

const int PWM_DRIVETRAIN_LEFT_MOTOR = 0;
const int PWM_DRIVETRAIN_RIGHT_MOTOR = 1;

//CAN IDs - Assigns names to the various CAN IDs
//EXAMPLE: const int CAN_PDB = 0;
/** \page motorID Motor Controller IDs
 * \verbatim
0 - PDB
1 - left drive motor
2 - right drive motor
Add more as needed.
 \endverbatim
 */

const int CAN_PDB = 19;
const int CAN_DRIVETRAIN_LEFT_MOTOR = 1;
const int CAN_DRIVETRAIN_LEFT_MOTOR_SLAVE = 2;
const int CAN_DRIVETRAIN_RIGHT_MOTOR = 3;
const int CAN_DRIVETRAIN_RIGHT_MOTOR_SLAVE = 4;
const int CAN_CLIMBER_MOTOR = 5;
const int CAN_CLIMBER_MOTOR_SLAVE = 6;
const int CAN_HOPPER_MOTOR = 7;
const int CAN_GEARINTAKE_MOTOR = 8;
const int CAN_FLOORINTAKEROLLER_MOTOR = 8;
const int CAN_FLOORINTAKEARM_MOTOR = 9;

//Relay Channels - Assigns names to Relay ports 1-8 on the roboRio
//EXAMPLE: const int RLY_COMPRESSOR = 1;

//Digital I/O - Assigns names to Digital I/O ports 1-14 on the roboRio
//EXAMPLE: const int DIO_DRIVETRAIN_BEAM_BREAK = 0;
const int DIO_ULTRASONIC_INPUT = 0;
const int DIO_ULTRASONIC_OUTPUT = 1;
const int DIO_PIXI = 3;

//Solenoid - Assigns names to Solenoid ports 1-8 on the roboRio
//EXAMPLE: const int SOL_DRIVETRAIN_SOLENOID_SHIFT_IN = 1;

//I2C - Assigns names to I2C ports 1-2 on the Roborio
//EXAMPLE: const int IO2C_AUTO_ACCEL = 1;

//Analog I/O - Assigns names to Analog I/O ports 1-8 on the roboRio
//EXAMPLE: const int AIO_BATTERY = 8;
const int AIO_PIXI = 0;

//Relay I/O - Assigns names to Realy I/O ports 1-8 on the roboRio
//EXAMPLE: const int RELAY_LED = 0;
const int RELAY_LED = 1;

//Joystick Input Device Counts - used by the listener to watch buttons and axis
const int JOYSTICK_BUTTON_COUNT = 10;
const int JOYSTICK_AXIS_COUNT = 5;

//POV IDs - Assign names to the 9 POV positions: -1 to 7
//EXAMPLE: const int POV_STILL = -1;
const int POV_STILL = -1;

//Primary Controller Mapping - Assigns action to buttons or axes on the first joystick
#undef	USE_X3D_FOR_CONTROLLER_1
#undef	USE_XBOX_FOR_CONTROLLER_1
#define	USE_L310_FOR_CONTROLLER_1

//Secondary Controller Mapping - Assigns action to buttons or axes on the second joystick
#undef	USE_X3D_FOR_CONTROLLER_2
#undef 	USE_XBOX_FOR_CONTROLLER_2
#define USE_L310_FOR_CONTROLLER_2

#ifdef USE_XBOX_FOR_CONTROLLER_1
#endif
/** \page joysticks Joystick Layouts
 * \verbatim
 	 +++++ Controller 1 +++++
  	A Button					Toggle noodle fan
  	B Button					~~
  	X Button					Hold Cube clicker at bottom to remove totes
  	Y Button					Release Cube clicker from hold
  	Start Button				Start Cube autocycle
  	Back Button					Stop Cube autocycle
  	Left Bumper					Run Conveyor forward
  	Right Bumper				Run Conveyor backwards - to claw
  	Left Thumbstick Button		Close CanLifter claw
  	Right Thumbstick Button		Open CanLifter claw
  	Left Thumbstick				Left tank, Arcade
  	Right Thumbstick			Right tank
  	D-pad						~~
  	Left Trigger				Lower CanLifter
  	RightTrigger				Raise CanLifter

 	 +++++ Controller 2 +++++
  	A Button					~~
  	B Button					~~
  	X Button					Hold Cube clicker at bottom to remove totes
  	Y Button					Release Cube clicker from hold
  	Start Button				Start Cube autocycle
  	Back Button					Stop Cube autocycle
  	Left Bumper					~~
  	Right Bumper				~~
 	Left Thumbstick Button		~~
  	Right Thumbstick Button		~~
  	Left Thumbstick				~~
  	Right Thumbstick			Raise/lower Cube clicker
  	D-pad						~~
  	Left Trigger				~~
  	RightTrigger				~~
 \endverbatim
 */
#ifdef USE_L310_FOR_CONTROLLER_1

#define TANK_DRIVE_LEFT				(pController_1->GetRawAxis(L310_THUMBSTICK_LEFT_Y))
#define TANK_DRIVE_RIGHT			(-pController_1->GetRawAxis(L310_THUMBSTICK_RIGHT_Y))
#define CHEEZY_DRIVE_WHEEL			(pController_1->GetRawAxis(L310_THUMBSTICK_RIGHT_X))
#define CHEEZY_DRIVE_THROTTLE		(-pController_1->GetRawAxis(L310_THUMBSTICK_LEFT_Y))
#define CHEEZY_DRIVE_SPIN		    (-pController_1->GetRawAxis(L310_TRIGGER_LEFT) + Controller_1->GetRawAxis(L310_TRIGGER_RIGHT))
#define CHEEZY_DRIVE_QUICKTURN		(pController_1->GetRawButton(L310_BUTTON_BUMPER_LEFT))

#define HOPPER_UP					(pController_1->GetRawButton(L310_BUTTON_A) || pController_2->GetRawButton(L310_BUTTON_A))
#define HOPPER_DOWN					(pController_1->GetRawButton(L310_BUTTON_B) || pController_2->GetRawButton(L310_BUTTON_B))

#define CLIMBER_UP					(pController_1->GetRawButton(L310_BUTTON_A) || pController_2->GetRawButton(L310_BUTTON_BUMPER_RIGHT))
#define CLIMBER_DOWN				0//(pController_1->GetRawButton(L310_BUTTON_B) || pController_2->GetRawButton(L310_BUTTON_BUMPER_LEFT))
/*
#define GEAR_INTAKE_HOLD			(pController_1->GetRawButton(L310_BUTTON_X) || pController_2->GetRawButton(L310_BUTTON_X))
#define GEAR_INTAKE_RELEASE			(pController_1->GetRawButton(L310_BUTTON_Y) || pController_2->GetRawButton(L310_BUTTON_Y))
*/
#define GEAR_POS_FLOOR				(pController_2->GetRawButton(L310_BUTTON_A))
#define GEAR_POS_SCORE				(pController_2->GetRawButton(L310_BUTTON_B))
#define GEAR_POS_ROBOT				(pController_2->GetRawButton(L310_BUTTON_Y))

#define GEAR_HANG_MACRO			    (pController_2->GetRawButton(L310_BUTTON_X))
#define GEAR_FLOOR_NEXTPOS			(pController_1->GetRawButton(L310_BUTTON_X) || pController_2->GetRawButton(L310_BUTTON_X))
#define GEAR_FLOOR_PREVPOS			(pController_1->GetRawButton(L310_BUTTON_Y) || pController_2->GetRawButton(L310_BUTTON_Y))
#define GEAR_FLOOR_PULLIN			/*(pController_1->GetRawAxis(L310_TRIGGER_RIGHT) +*/(pController_2->GetRawAxis(L310_TRIGGER_LEFT))
#define GEAR_FLOOR_PUSHOUT			/*(pController_1->GetRawAxis(L310_TRIGGER_LEFT) +*/ (pController_2->GetRawAxis(L310_TRIGGER_RIGHT))

#define PIXIE_LIGHT					(pController_1->GetRawButton(L310_BUTTON_START) || pController_2->GetRawButton(L310_BUTTON_START))

#endif // USE_L310_FOR_CONTROLLER_1

#ifdef USE_X3D_FOR_CONTROLLER_2
#endif // USE_X3D_FOR_CONTROLLER_2

#ifdef USE_XBOX_FOR_CONTROLLER_2
#endif // USE_XBOX_FOR_CONTROLLER_2

#ifdef USE_L310_FOR_CONTROLLER_2
#endif // USE_L310_FOR_CONTROLLER_2

#endif //ROBOT_PARAMS_H
