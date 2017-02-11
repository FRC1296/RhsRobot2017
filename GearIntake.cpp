///** \file
// * Example of subsystem task behavior.
// *
// * This class is derived from the standard Component base class and includes
// * initialization for the devices used to control a given subsystem.
// *
// * The task receives messages from the main robot class and implements behaviors
// * for a given subsystem.
// */
//
//#include <CANTalon.h>
//#include <GearIntake.h>
//#include <ComponentBase.h>
//#include <RobotParams.h>
//#include "WPILib.h"
//
////Robot
//
//GearIntake::GearIntake()
//: ComponentBase(GEARINTAKE_TASKNAME, GEARINTAKE_QUEUE, GEARINTAKE_PRIORITY)
//{
//	pGearIntakeMotor1 = new CANTalon(CAN_GEARINTAKE_MOTOR1);
//
//	pGearIntakeMotor1->SetVoltageRampRate(48.0);
//
//	wpi_assert(pGearIntakeMotor1);
//
//	pGearIntakeMotor1->ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
//	pGearIntakeMotor1->SetControlMode(CANTalon::kPercentVbus);
//
//	pTask = new std::thread(&Component::StartTask, this, GEARINTAKE_TASKNAME, GEARINTAKE_PRIORITY);
//	wpi_assert(pTask);
//};
//
//GearIntake::~GearIntake()
//{
//	//TODO delete member objects
//	delete(pTask);
//	delete pGearIntakeMotor1;
//};
//
//void GearIntake::OnStateChange()
//{
//	switch(localMessage.command)
//			{
//				case COMMAND_ROBOT_STATE_AUTONOMOUS:
//					break;
//
//				case COMMAND_ROBOT_STATE_TEST:
//					break;
//
//				case COMMAND_ROBOT_STATE_TELEOPERATED:
//					break;
//
//				case COMMAND_ROBOT_STATE_DISABLED:
//					break;
//
//				case COMMAND_ROBOT_STATE_UNKNOWN:
//					break;
//
//				default:
//					break;
//			}
//};
//
//void GearIntake::Run()
//{
//	switch(localMessage.command)			//Reads the message command
//	{
//	/*	case COMMAND_GEARINTAKE_RELEASE:
//						pGearIntakeMotor1->Set(localMessage.params.gearintake.GearIntakeRelease*-1);
//						break;
//
//		case COMMAND_GEARINTAKE_HOLD:
//						pGearIntakeMotor1->Set(localMessage.params.gearintake.GearIntakeHold*-1);
//						break;
//
//	    case COMMAND_GEARINTAKE_STOP:
//						pGearIntakeMotor1->Set(0);
//						break;
//
//	    case COMMAND_SYSTEM_MSGTIMEOUT:  // what should we do if we do not get a timely message?
//						break;
//
//		default:
//			break;*/
//		}
//};
