/** \file
 * Main robot class.
 *
 * The RhsRobot class is the main robot class. It inherits from RhsRobotBase and MUST define the Init() function, the Run() function, and
 * the OnStateChange() function.  Messages from the DS are processed and commands.
 */
#ifndef RHS_ROBOT_H
#define RHS_ROBOT_H

#include "Autonomous.h"
#include "Drivetrain.h"
#include "Hopper.h"
#include "Climber.h"
#include "GearIntake.h"
#include "GearFloorIntake.h"

#include <RhsRobotBase.h>
#include "WPILib.h"

class RhsRobot : public RhsRobotBase
{
public:
	RhsRobot();
	virtual ~RhsRobot();

private:
	Joystick* pController_1;
	Joystick* pController_2;
	Drivetrain* pDrivetrain;
	Autonomous* pAutonomous;
	Climber* pClimber;
	Hopper* pHopper;
	GearIntake* pGearIntake;
	GearFloorIntake* pGearFloor;

	cs::UsbCamera camera;

	std::vector <ComponentBase *> ComponentSet;
	
	void Init();
	void OnStateChange();
	void Run();

	bool bHopperRunning;
	bool bGearButtonDown;
	int iLoop;
};

#endif //RHS_ROBOT_H
