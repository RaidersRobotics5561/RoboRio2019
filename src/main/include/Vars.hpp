/*
  Vars.hpp

   Created on: Feb 14, 2018
       Author: 5561
 */
#ifndef SRC_ROBORIO2018_VARS_HPP_
#define SRC_ROBORIO2018_VARS_HPP_

#include "enums.hpp"

extern double      V_LukeStopperRamp;
extern double      V_RotateGain;

extern double      V_EndGameWinchTime;
extern bool        V_LED_RainbowLatch;
extern int         V_AutonState;

extern double      V_WheelSpeedErrorPrev[E_RobotSideSz];
extern double      V_WheelSpeedErrorIntegral[E_RobotSideSz];
extern double      V_WheelRPM_Raw[E_RobotSideSz];
extern double      V_WheelRPM_Filt[E_RobotSideSz];
extern double      V_WheelRPM_FiltPrev[E_RobotSideSz];
extern double      V_WheelRPM_Desired[E_RobotSideSz];
extern double      V_WheelMotorCmndPct[E_RobotSideSz];

extern double      V_UltraSonicDistance[E_RobotSideSz];

extern double      V_IntakePID_Gain[E_PID_Sz];

extern double      V_RobotMotorCmndPct[E_RobotMotorSz];
extern double      V_WheelProportionalGain[E_RobotSideSz];
extern double      V_WheelIntegralGain[E_RobotSideSz];
extern double      V_WheelDerivativeGain[E_RobotSideSz];
extern double      V_WheelSpeedLagFiltGain[E_RobotSideSz];

extern double      V_HookPosition;
extern double      V_HookPositionErrorPrev;
extern double      V_HookPositionErrorIntegral;
extern double      V_HookLiftHeightDesired;
extern double      V_HookPID_Gain[E_PID_Sz];

extern double      V_IntakeLiftHeightDesired;
extern double      V_IntakePositionPrev;
extern double      V_IntakePosition;
extern double      V_IntakePositionErrorPrev;
extern double      V_IntakePositionErrorIntegral;

extern double      V_GyroAngleRelative;
extern double      V_GyroAngleOffset;
extern double      V_ArmAngleDeg;
extern T_DriveMode DriveMode;
extern double      V_Revolutions[E_RobotSideSz];
extern double      V_DistanceTraveled[E_RobotSideSz];
extern double      V_DistanceTraveledAvg;
extern double      V_IntakeArmPulseToRev[E_ArmCmndSz];
extern double      V_RobotUserCmndPct[E_RobotUserCmndSz];

extern double      V_RollerTimer;
extern double      V_IntakeArmTimer;

extern double      V_AutonWheelDebounceTimer[E_RobotSideSz];
extern double      V_AutonRotateDebounceTimer;
extern T_RobotSide V_AutonTargetSide[3];

extern double      V_AutonIntakeLiftDebounceTimer;


#endif /* SRC_ROBORIO2018_VARS_HPP_ */
