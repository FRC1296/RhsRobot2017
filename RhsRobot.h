/** \file
 * Main robot class.
 *
 * The RhsRobot class is the main robot class. It inherits from RhsRobotBase and MUST define the Init() function, the Run() function, and
 * the OnStateChange() function.  Messages from the DS are processed and commands.
 */
#ifndef RHS_ROBOT_H
#define RHS_ROBOT_H

#include <Autonomous.h>
#include <Drivetrain.h>
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

	std::vector <ComponentBase *> ComponentSet;
	
	void Init();
	void OnStateChange();
	void Run();
	bool CheckButtonPressed(bool, bool);
	bool CheckButtonReleased(bool, bool);


	int iLoop;
};

#endif //RHS_ROBOT_H
