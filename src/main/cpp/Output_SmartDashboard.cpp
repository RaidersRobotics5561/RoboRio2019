/*
  Output_SmartDashboard.cpp

   Created on: Feb 11, 2018
       Author: 5561
 */

#include "Output_SmartDashboard.hpp"


/******************************************************************************
 * Function:     UpdateSmartDashboad
 *
 * Description:  Report data back to the smart dashboard.
 ******************************************************************************/
void UpdateSmartDashboad(void)
  {
    SmartDashboard::PutNumber("SpeedRawLeft", V_WheelRPM_Raw[E_RobotSideLeft]);
    SmartDashboard::PutNumber("SpeedRawRight", V_WheelRPM_Raw[E_RobotSideRight]);
    SmartDashboard::PutNumber("RightSide", V_RobotMotorCmndPct[E_RobotMotorRightWheel]);
    SmartDashboard::PutNumber("LeftSide", V_RobotMotorCmndPct[E_RobotMotorLeftWheel]);
    SmartDashboard::PutNumber("LeftDesired", V_WheelRPM_Desired[E_RobotSideLeft]);
    SmartDashboard::PutNumber("RightDesired", V_WheelRPM_Desired[E_RobotSideRight]);

    SmartDashboard::PutNumber("DriveMode",(double)DriveMode);

    SmartDashboard::PutNumber("GyroRelative",V_GyroAngleRelative);

    SmartDashboard::PutNumber("DistanceRight", V_DistanceTraveled[E_RobotSideRight]);
    SmartDashboard::PutNumber("DistanceLeft", V_DistanceTraveled[E_RobotSideLeft]);

    SmartDashboard::PutNumber("Hook Position",     V_HookPosition);
    SmartDashboard::PutNumber("Hook Desired",     V_HookLiftHeightDesired);
    SmartDashboard::PutNumber("Hook Motor", V_RobotMotorCmndPct[E_RobotMotorHook]);

    SmartDashboard::PutNumber("Intake Cmnd",   V_RobotUserCmndPct[E_RobotUserCmndLift]);
    SmartDashboard::PutNumber("Intake Position",   V_IntakePosition);
    SmartDashboard::PutNumber("Intake Desired Position",   V_IntakeLiftHeightDesired);
    SmartDashboard::PutNumber("Intake Lift Motor", V_RobotMotorCmndPct[E_RobotMotorLift]);

    SmartDashboard::PutNumber("UltraRight", V_UltraSonicDistance[E_RobotSideRight]);
    SmartDashboard::PutNumber("UltraLeft", V_UltraSonicDistance[E_RobotSideLeft]);

    SmartDashboard::PutNumber("V_UltraSonicDistance[E_RobotSideRight]", V_UltraSonicDistance[E_RobotSideRight]);
    SmartDashboard::PutNumber("V_UltraSonicDistance[E_RobotSideLeft]", V_UltraSonicDistance[E_RobotSideLeft]);
  };
