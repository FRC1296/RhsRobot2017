/** \file
 * Main robot class.
 *
 * The RhsRobot class is the main robot class. It inherits from RhsRobotBase and MUST define the Init() function, the Run() function, and
 * the OnStateChange() function.  Messages from the DS are processed and commands sent to the subsystems
 * that implement behaviors for each part for the robot.
 */

#include <ComponentBase.h>
#include <RhsRobot.h>
#include <RobotParams.h>
#include "WPILib.h"

// The constructor sets the pointer to our objects to NULL.  We use pointers so we
// can control when the objects are instantiated.  It looks kinda old school but is
// standard practice in embedded systems.

RhsRobot::RhsRobot() {
	pController_1 = NULL;  // driver number one's joystick
	pController_2 = NULL;  // driver number two's joystick
	pAutonomous = NULL;    // the object that executes anonymous behaviours
	pDrivetrain = NULL;    // the object that drives the wheels
	pClimber = NULL;
	//pHopper =NULL;

    // set new object pointers to NULL here

	iLoop = 0;            // a helpful little loop counter
}

// This will never get used but we make it functional anyways.  We iterate through our objects
// (with messaging) then delete the system objects.

RhsRobot::~RhsRobot() {
	std::vector<ComponentBase *>::iterator nextComponent = ComponentSet.begin();

	for(; nextComponent != ComponentSet.end(); ++nextComponent)
	{
		delete (*nextComponent);
	}

	delete pController_1;
	delete pController_2;

	// delete other system objects here (but not our message-based objects)
}

void RhsRobot::Init() {
	/* 
	 * Set all pointers to null and then allocate memory and construct objects
	 * EXAMPLE:	drivetrain = NULL; (in constructor)
	 * 			drivetrain = new Drivetrain(); (in RhsRobot::Init())
	 */
	pController_1 = new Joystick(0);
	pController_2 = new Joystick(1);
	pDrivetrain = new Drivetrain();
	pAutonomous = new Autonomous();
	pClimber    = new Climber();
	//pHopper = new Hopper();

	std::vector<ComponentBase *>::iterator nextComponent = ComponentSet.begin();

	if(pAutonomous)
	{
		nextComponent = ComponentSet.insert(nextComponent, pAutonomous);
	}

	if(pDrivetrain)
	{
		nextComponent = ComponentSet.insert(nextComponent, pDrivetrain);
	}

	if(pHopper)
	{
		nextComponent = ComponentSet.insert(nextComponent, pHopper);
	}

	if(pClimber)
	{
		nextComponent = ComponentSet.insert(nextComponent, pClimber);
	}
	// instantiate our other objects here
}

// this method iterates through all our objects (in our message infrastructure) and sends
// them a message, it is used mostly for telling every object the robot state has changed

void RhsRobot::OnStateChange() {
	std::vector<ComponentBase *>::iterator nextComponent;

	for(nextComponent = ComponentSet.begin();
			nextComponent != ComponentSet.end(); ++nextComponent)
	{
		(*nextComponent)->SendMessage(&robotMessage);
	}
}

// this method is where the magic happens.  t is called every time we get a new message from th driver station

void RhsRobot::Run() {
	/* Poll for control data and send messages to each subsystem. Surround blocks with if(component) so entire components can be disabled
	 * by commenting out their construction.
	 * EXAMPLE: if(pDrivetrain)
	 * 			{ 
	 * 				// Check joysticks and send messages
	 * 			}
	 */

	if(pAutonomous)
	{
		if(GetCurrentRobotState() == ROBOT_STATE_AUTONOMOUS)
		{
			// all messages to components will come from the autonomous task
			return;
		}
	}

	if (pDrivetrain)
	{
		if (TANK_DRIVE_STOP)
		{
			robotMessage.command = COMMAND_DRIVETRAIN_STOP;
			pDrivetrain->SendMessage(&robotMessage);
		}

		else
		{
			/*robotMessage.command = COMMAND_DRIVETRAIN_DRIVE_TANK;
			robotMessage.params.tankDrive.left = TANK_DRIVE_LEFT;
			robotMessage.params.tankDrive.right = TANK_DRIVE_RIGHT;*/
			robotMessage.command = COMMAND_DRIVETRAIN_DRIVE_ARCADE;
			robotMessage.params.arcadeDrive.vertical = ARCADE_DRIVE_VERTICAL;
			robotMessage.params.arcadeDrive.horizontal = ARCADE_DRIVE_HORIZONTAL;
			//robotMessage.params.arcadeDrive.velocity = ;
			//robotMessage.params.arcadeDrive.position = ;
			pDrivetrain->SendMessage(&robotMessage);
		}
	}
	if (pHopper)
	{
		if (HOPPER_UP)
		{
			robotMessage.command = COMMAND_HOPPER_UP;
			robotMessage.params.hopper.HopUp = 1.0;
			pHopper->SendMessage(&robotMessage);
		}

		else if (HOPPER_DOWN)
		{
			robotMessage.command = COMMAND_HOPPER_DOWN;
			robotMessage.params.hopper.HopDown = 1.0;
			pHopper->SendMessage(&robotMessage);
		}

		else
		{
			robotMessage.command = COMMAND_HOPPER_STOP;
			pClimber->SendMessage(&robotMessage);
		}

	}

	if (pClimber)
	{
		if (CLIMBER_UP)
		{
			robotMessage.command = COMMAND_CLIMBER_UP;
			robotMessage.params.climber.ClimbUp = 1.0;
			pClimber->SendMessage(&robotMessage);
		}
		else if (CLIMBER_DOWN)
		{
			robotMessage.command = COMMAND_CLIMBER_DOWN;
			robotMessage.params.climber.ClimbDown = -.2;
			pClimber->SendMessage(&robotMessage);
		}
		else
		{
			robotMessage.command = COMMAND_CLIMBER_STOP;
			pClimber->SendMessage(&robotMessage);
		}

	}

}

START_ROBOT_CLASS(RhsRobot)
