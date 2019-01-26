/*
  Control_Auton.cpp

   Created on: Feb 23, 2018
       Author: 5561
 */

#include "Enums.hpp"
#include "const.h"
#include "Control_Auton.hpp"
#include "Calibrations.hpp"
#include "Vars.hpp"
#include "Control_PID.hpp"

double V_RollerTimer;
double V_IntakeArmTimer;


/******************************************************************************
 * Function:     DtrmnAutonOption
 *
 * Description:  Determine what path the robot will travel in during the auton
 *               mode.  This should only need to be called once when starting
 *               auton mode.:
 *
 *                   E_AutonOpt0  - LLL
 *                   E_AutonOpt1  - LLR
 *                   E_AutonOpt2  - RLL
 *                   E_AutonOpt3  - RLR
 *                   E_AutonOpt4  - MLL
 *                   E_AutonOpt5  - MLR
 *                   E_AutonOpt6  - LLF
 *                   E_AutonOpt7  - LLS
 *                   E_AutonOpt8  - LRS
 *                   E_AutonOpt9  - LRF
 *                   E_AutonOpt10 - MLF
 *                   E_AutonOpt11 - MLS
 *                   E_AutonOpt12 - MRS
 *                   E_AutonOpt13 - MRF
 *                   E_AutonOpt14 - RLF
 *                   E_AutonOpt15 - RLS
 *                   E_AutonOpt16 - RRS
 *                   E_AutonOpt17 - RRF
 *
 ******************************************************************************/
T_AutonOpt DtrmnAutonOption(T_RobotSide       L_AutonTargetSwitch,
                            T_RobotSide       L_AutonTargetScale,
                            T_AutonStartPos   L_AutonStartPos,
                            T_AutonPreference L_AutonPreference)
  {
  T_AutonOpt L_AutonOption;

  if (L_AutonStartPos == E_AutonStartPosLeft)
    {
    if (((L_AutonTargetScale == E_RobotSideLeft) &&
         (L_AutonPreference == E_AutonPreferenceScale)) ||

        ((L_AutonTargetScale  == E_RobotSideLeft) &&
         (L_AutonTargetSwitch == E_RobotSideRight)))
      {
      L_AutonOption = E_AutonOpt1;
      }
    else if (((L_AutonTargetSwitch == E_RobotSideLeft) &&
             (L_AutonPreference == E_AutonPreferenceSwitch)) ||

            ((L_AutonTargetSwitch  == E_RobotSideLeft) &&
             (L_AutonTargetScale   == E_RobotSideRight)))
      {
      L_AutonOption = E_AutonOpt0;
      }
    else
      {
      L_AutonOption = E_AutonOpt6; // Default
      }
    }
  else if (L_AutonStartPos == E_AutonStartPosMiddle)
    {
    if (L_AutonTargetSwitch == E_RobotSideLeft)
      {
      L_AutonOption = E_AutonOpt2;
      }
    else if (L_AutonTargetSwitch == E_RobotSideRight)
      {
      L_AutonOption = E_AutonOpt3;
      }
    else
      {
      L_AutonOption = E_AutonOpt6; // Default, we don't have the time to reach the scale when in the middle position...
      }
    }
  else if (L_AutonStartPos == E_AutonStartPosRight)
    {
    if (((L_AutonTargetScale == E_RobotSideRight) &&
        (L_AutonPreference == E_AutonPreferenceScale)) ||

       ((L_AutonTargetScale  == E_RobotSideRight) &&
        (L_AutonTargetSwitch == E_RobotSideLeft)))
      {
      L_AutonOption = E_AutonOpt4;
      }
    else if (((L_AutonTargetSwitch == E_RobotSideRight) &&
              (L_AutonPreference == E_AutonPreferenceSwitch)) ||

             ((L_AutonTargetSwitch  == E_RobotSideRight) &&
              (L_AutonTargetScale   == E_RobotSideLeft)))
      {
      L_AutonOption = E_AutonOpt5;
      }
    else
      {
      L_AutonOption = E_AutonOpt6; // Default
      }
    }
  else // L_AutonStartPos == E_AutonStartPosDefault
    {
    L_AutonOption = E_AutonOpt6; // Default
    }

  return (L_AutonOption);
  }

/******************************************************************************
 * Function:     DtrmnActuatorComplete
 *
 * Description:  Determine when an actuator has reached its desired end point.
 *
 ******************************************************************************/
bool DtrmnActuatorComplete(double     L_CntrlVal,
                           double     L_MeasuredVal,
                           double     L_Deadband,
                           double    *L_DebounceTimer,
                           double     L_DebounceThreshold)
  {
  bool   L_ControlComplete = false;
  double L_DeadbandHigh;
  double L_DeadbandLow;

  L_DeadbandHigh = L_CntrlVal + L_Deadband;
  L_DeadbandLow  = L_CntrlVal - L_Deadband;

  if ((L_MeasuredVal >= L_DeadbandLow) &&
      (L_MeasuredVal <= L_DeadbandHigh))
    {
    *L_DebounceTimer += C_ExeTime;

    if (*L_DebounceTimer >= L_DebounceThreshold)
      {
      *L_DebounceTimer = L_DebounceThreshold;
      L_ControlComplete = true;
      }
    }
  else
    {
    *L_DebounceTimer = 0.0;
    }

  return (L_ControlComplete);
  }



/******************************************************************************
 * Function:     CntrlAutonDesiredSpeed
 *
 * Description:  Determine the desired speed for a given actuator(s) while in
 *               Auton.  The intent is to ramp the requested speed up from zero,
 *               to a max value, then ramp down to a minimum value once we get
 *               near the end.
 *
 ******************************************************************************/
double CntrlAutonDesiredSpeed(double            L_K_TotalPlannedTravel,
                              double            L_MeasuredTravel,
                              double            L_DesiredSpeedPrev,
                              double            L_K_RampDownTravel,
                              double            L_K_MinSpeed,
                              double            L_K_MaxSpeed,
                              double            L_K_SpeedRamp,
                              double            L_DebounceTime,
                              bool              L_TargetMet)
  {
  double L_DesiredSpeed     = 0.0;
  double L_RampDownDistance = 0.0;
  double L_DesiredDirectionSign = 1.0;
  double L_ActualDirectionSign  = 1.0;

  /* Find the angle at which we want to start ramping out the desired speed. */
  L_RampDownDistance = fabs(L_K_TotalPlannedTravel) - fabs(L_K_RampDownTravel);

  if (L_RampDownDistance < 0.0)
    {
    /* Limit to zero */
    L_RampDownDistance = 0.0;
    }

  if (L_K_TotalPlannedTravel < 0)
    {
    L_DesiredDirectionSign = -1.0;
    }

  if (L_MeasuredTravel < 0)
    {
    L_ActualDirectionSign = -1.0;
    }

  if (fabs(L_MeasuredTravel) >= L_RampDownDistance)
    {
    /* We have reached the point at which we need to start slowing down
     * (i.e. let's put the brakes on!). */
    L_DesiredSpeed = L_K_MinSpeed * L_DesiredDirectionSign;
    }
  else if (fabs(L_MeasuredTravel) < L_RampDownDistance)
    {
    /* We still have a way to go.
     * Let's speed up! */
    L_DesiredSpeed = L_K_MaxSpeed * L_DesiredDirectionSign;
    }

  if ((L_TargetMet == true) ||
      (L_DebounceTime > 0.0))
    {
    /* We have met our target or the debounce timers are running.
     * Let's request 0 speed. */
    L_DesiredSpeed = 0.0;
    }
  else if ((L_DesiredDirectionSign > 0.0) &&
           (L_MeasuredTravel > L_K_TotalPlannedTravel))
    {
    /*Opps!  Looks like we overshot our target.  Lets back up.*/
    L_DesiredSpeed = -L_K_MinSpeed;
    }
  else if ((L_DesiredDirectionSign < 0.0) &&
           (L_MeasuredTravel < L_K_TotalPlannedTravel))
    {
    /*Opps!  Looks like we overshot our target.  Lets back up.*/
    L_DesiredSpeed = L_K_MinSpeed;
    }
  else if (L_DesiredSpeed < L_K_MinSpeed)
    {
    L_DesiredSpeed = L_K_MinSpeed * L_DesiredDirectionSign;
    }

  return (L_DesiredSpeed);
  }

/******************************************************************************
 * Function:     CntrlAutonDesiredSpeed
 *
 * Description:  Determine the desired speed for a given actuator(s) while in
 *               Auton.  The intent is to ramp the requested speed up from zero,
 *               to a max value, then ramp down to a minimum value once we get
 *               near the end.
 *
 ******************************************************************************/
double CntrlAutonUltraSonic(double            L_K_TotalPlannedTravel,
                            double            L_MeasuredTravel,
                            double            L_K_MinSpeed,
                            bool              L_TargetMet)
  {
  double L_DesiredSpeed     = 0.0;
  double L_Error = 0.0;

  L_Error = L_MeasuredTravel - L_K_TotalPlannedTravel;

  if (L_Error > 0)
    {
    L_DesiredSpeed = L_K_MinSpeed;
    }
  else if (L_Error < 0)
    {
    L_DesiredSpeed = -L_K_MinSpeed;
    }

  if (L_TargetMet == true)
    {
    L_DesiredSpeed = 0.0;
    }

  return (L_DesiredSpeed);
  }

/******************************************************************************
 * Function:     CntrlAutonDesiredRotate
 *
 * Description:  Determine the desired speed for a given actuator(s) while in
 *               Auton.  The intent is to ramp the requested speed up from zero,
 *               to a max value, then ramp down to a minimum value once we get
 *               near the end.
 *
 ******************************************************************************/
double CntrlAutonDesiredRotate(double           L_K_TotalPlannedRotation,
                              double            L_MeasuredRotation,
                              double            L_DesiredRotationPrev,
                              double            L_K_ProportionalGx,
                              double            L_K_MinSpeed,
                              double            L_K_MaxSpeed,
                              double            L_DebounceTime,
                              bool              L_TargetMet)
  {
  double L_DesiredSpeed     = 0.0;
  double L_Error            = 0.0;
  double L_Direction        = 0.0;

  L_Error = L_K_TotalPlannedRotation - L_MeasuredRotation;

  if (L_Error > 0)
    {
    L_Direction = 1.0;
    }
  else if (L_Error < 0)
    {
    L_Direction = -1.0;
    }

  L_DesiredSpeed = L_Error * L_K_ProportionalGx;

  if (fabs(L_DesiredSpeed) > L_K_MaxSpeed)
    {
    L_DesiredSpeed = L_K_MaxSpeed * L_Direction;
    }
  else if (fabs(L_DesiredSpeed) < L_K_MinSpeed)
    {
    L_DesiredSpeed = L_K_MinSpeed * L_Direction;
    }



  if ((L_TargetMet == true) ||
      (L_DebounceTime > 0.0))
    {
    L_DesiredSpeed = 0.0;
    }

  return (L_DesiredSpeed);
  }


/******************************************************************************
 * Function:     CntrlAutonDesiredLift
 *
 * Description:  Determine the desired speed for a given actuator(s) while in
 *               Auton.  The intent is to ramp the requested speed up from zero,
 *               to a max value, then ramp down to a minimum value once we get
 *               near the end.
 *
 ******************************************************************************/
double CntrlAutonDesiredLift(double             L_K_TotalPlannedTravel,
                              double            L_MeasuredTravel,
                              double            L_DesiredHeightPrev,
                              double            L_K_DesiredTravelRamp,
                              bool              L_TargetMet)
  {
  double L_DesiredHeight    = L_DesiredHeightPrev;


  if (L_TargetMet == true)
    {
    L_DesiredHeight    = L_K_TotalPlannedTravel;
    }
  else if ((L_MeasuredTravel > L_K_TotalPlannedTravel) ||
           (L_DesiredHeightPrev > L_K_TotalPlannedTravel))
    {
    L_DesiredHeight = L_DesiredHeightPrev - (L_K_DesiredTravelRamp * C_ExeTime);
    if (L_DesiredHeight < L_K_TotalPlannedTravel)
      {
      L_DesiredHeight = L_K_TotalPlannedTravel;
      }
    }
  else if ((L_MeasuredTravel < L_K_TotalPlannedTravel) ||
           (L_DesiredHeightPrev < L_K_TotalPlannedTravel))
    {
    L_DesiredHeight = L_DesiredHeightPrev + (L_K_DesiredTravelRamp * C_ExeTime);
    if (L_DesiredHeight > L_K_TotalPlannedTravel)
      {
      L_DesiredHeight = L_K_TotalPlannedTravel;
      }
    }

  if (L_DesiredHeight >= K_MaxIntakeLiftHeight)
    {
    L_DesiredHeight = K_MaxIntakeLiftHeight;
    }

  return (L_DesiredHeight);
  }


/******************************************************************************
 * Function:     CntrlAutonDrive
 *
 * Description:  Control the drivetrain in auton.
 *
 ******************************************************************************/
bool CntrlAutonDrive(T_Actuator       L_CntrlActuator,
                     double           L_AutonTarget)
  {
  bool        L_ControlComplete = false;
  T_RobotSide L_RobotSide       = E_RobotSideLeft;
  float       L_RobotPID_Gx[E_RobotSideSz][E_PID_Sz];

  L_RobotPID_Gx[E_RobotSideLeft][E_PID_Proportional]  = K_WheelSpeedPID_GainAuton[E_RobotSideLeft][E_PID_Proportional];
  L_RobotPID_Gx[E_RobotSideLeft][E_PID_Integral]      = K_WheelSpeedPID_GainAuton[E_RobotSideLeft][E_PID_Integral];
  L_RobotPID_Gx[E_RobotSideLeft][E_PID_Derivative]    = K_WheelSpeedPID_GainAuton[E_RobotSideLeft][E_PID_Derivative];
  L_RobotPID_Gx[E_RobotSideRight][E_PID_Proportional] = K_WheelSpeedPID_GainAuton[E_RobotSideRight][E_PID_Proportional];
  L_RobotPID_Gx[E_RobotSideRight][E_PID_Integral]     = K_WheelSpeedPID_GainAuton[E_RobotSideRight][E_PID_Integral];
  L_RobotPID_Gx[E_RobotSideRight][E_PID_Derivative]   = K_WheelSpeedPID_GainAuton[E_RobotSideRight][E_PID_Derivative];

  if (L_CntrlActuator == E_ActuatorDriveEncoder)
    {
    L_ControlComplete = DtrmnActuatorComplete( L_AutonTarget,
                                               V_DistanceTraveledAvg,
                                               K_AutonDriveDistanceDeadband,
                                              &V_AutonWheelDebounceTimer[E_RobotSideRight],
                                               K_AutonDebounceThreshold);

    V_WheelRPM_Desired[E_RobotSideRight] = CntrlAutonDesiredSpeed(L_AutonTarget,
                                                                     V_DistanceTraveledAvg,
                                                                     V_WheelRPM_Desired[E_RobotSideRight],
                                                                     K_AutonDriveDistanceToSlow,
                                                                     K_AutonDriveMinSpeed,
                                                                     K_AutonDriveMaxSpeed,
                                                                     K_AutonDriveSpeedRamp,
                                                                     V_AutonWheelDebounceTimer[E_RobotSideRight],
                                                                     L_ControlComplete);

    /* Since we are driving straight, we will have a desired speed that will be the same for both sides: */
    V_WheelRPM_Desired[E_RobotSideLeft] = V_WheelRPM_Desired[E_RobotSideRight];
    }
  else if (L_CntrlActuator == E_ActuatorDriveUltraSonic)
    {
    for (L_RobotSide = E_RobotSideLeft;
         L_RobotSide < E_RobotSideSz;
         L_RobotSide = T_RobotSide(int(L_RobotSide) + 1))
      {
      L_ControlComplete = DtrmnActuatorComplete( L_AutonTarget,
                                                 V_UltraSonicDistance[L_RobotSide],
                                                 K_AutonDriveDistanceUltraDeadband,
                                                &V_AutonWheelDebounceTimer[L_RobotSide],
                                                 K_AutonDebounceThreshold);

      V_WheelRPM_Desired[L_RobotSide] = CntrlAutonUltraSonic(L_AutonTarget,
                                                             V_UltraSonicDistance[L_RobotSide],
                                                             K_AutonDriveMinSpeedUltra,
                                                             L_ControlComplete);

      }
    }
  else if (L_CntrlActuator == E_ActuatorRotate)
    {
    L_RobotPID_Gx[E_RobotSideLeft][E_PID_Proportional]  = K_WheelSpeedPID_GainRotateAuton[E_RobotSideLeft][E_PID_Proportional];
    L_RobotPID_Gx[E_RobotSideLeft][E_PID_Integral]      = K_WheelSpeedPID_GainRotateAuton[E_RobotSideLeft][E_PID_Integral];
    L_RobotPID_Gx[E_RobotSideLeft][E_PID_Derivative]    = K_WheelSpeedPID_GainRotateAuton[E_RobotSideLeft][E_PID_Derivative];
    L_RobotPID_Gx[E_RobotSideRight][E_PID_Proportional] = K_WheelSpeedPID_GainRotateAuton[E_RobotSideRight][E_PID_Proportional];
    L_RobotPID_Gx[E_RobotSideRight][E_PID_Integral]     = K_WheelSpeedPID_GainRotateAuton[E_RobotSideRight][E_PID_Integral];
    L_RobotPID_Gx[E_RobotSideRight][E_PID_Derivative]   = K_WheelSpeedPID_GainRotateAuton[E_RobotSideRight][E_PID_Derivative];

    L_ControlComplete = DtrmnActuatorComplete( L_AutonTarget,
                                               V_GyroAngleRelative,
                                               K_AutonRotateAngleDeadband,
                                              &V_AutonRotateDebounceTimer,
                                               K_AutonDebounceThreshold);

    V_WheelRPM_Desired[E_RobotSideLeft] = CntrlAutonDesiredRotate(L_AutonTarget,
                                                                 V_GyroAngleRelative,
                                                                 V_WheelRPM_Desired[E_RobotSideLeft],
                                                                 K_AutonRotateAnglePropGx,
                                                                 K_AutonRotateMinSpeed,
                                                                 K_AutonRotateMaxSpeed,
                                                                 V_AutonRotateDebounceTimer,
                                                                 L_ControlComplete);


    /* Since we are rotating while in tank drive, we will have a desired speed that is opposite from the other side: */
    V_WheelRPM_Desired[E_RobotSideRight] = -V_WheelRPM_Desired[E_RobotSideLeft];
    }


  for (L_RobotSide = E_RobotSideLeft;
       L_RobotSide < E_RobotSideSz;
       L_RobotSide = T_RobotSide(int(L_RobotSide) + 1))
    {
    V_RobotMotorCmndPct[L_RobotSide]  = Control_PID(V_WheelRPM_Desired[L_RobotSide],
                                                    V_WheelRPM_Filt[L_RobotSide],
                                                    &V_WheelSpeedErrorPrev[L_RobotSide],
                                                    &V_WheelSpeedErrorIntegral[L_RobotSide],
                                                    L_RobotPID_Gx[L_RobotSide][E_PID_Proportional],
                                                    L_RobotPID_Gx[L_RobotSide][E_PID_Integral],
                                                    L_RobotPID_Gx[L_RobotSide][E_PID_Derivative],
                                                    K_WheelSpeedProportionalLimit[L_RobotSide][E_IntergalUpperLimit],
                                                    K_WheelSpeedProportionalLimit[L_RobotSide][E_IntergalLowerLimit],
                                                    K_WheelSpeedIntergalLimit[L_RobotSide][E_IntergalUpperLimit],
                                                    K_WheelSpeedIntergalLimit[L_RobotSide][E_IntergalLowerLimit],
                                                    K_WheelSpeedDerivativeLimit[L_RobotSide][E_IntergalUpperLimit],
                                                    K_WheelSpeedDerivativeLimit[L_RobotSide][E_IntergalLowerLimit],
                                                    K_WheelSpeedCmndLimit[L_RobotSide][E_IntergalUpperLimit],
                                                    K_WheelSpeedCmndLimit[L_RobotSide][E_IntergalLowerLimit]);
    }


  return (L_ControlComplete);
  }

/******************************************************************************
 * Function:     CntrlAutonLift
 *
 * Description:  Control the lift in auton.
 *
 ******************************************************************************/
bool CntrlAutonLift(T_Actuator       L_CntrlActuator,
                    double           L_AutonTarget)
  {
  bool        L_ControlComplete = false;

  L_ControlComplete = DtrmnActuatorComplete( L_AutonTarget,
                                             V_IntakePosition,
                                             K_AutonIntakeDistanceDeadband,
                                            &V_AutonIntakeLiftDebounceTimer,
                                             K_AutonDebounceThreshold);

  V_IntakeLiftHeightDesired = CntrlAutonDesiredLift(L_AutonTarget,
                                                    V_IntakePosition,
                                                    V_IntakeLiftHeightDesired,
                                                    K_AutonIntakeRamp,
                                                    L_ControlComplete);

  return (L_ControlComplete);
  }

/******************************************************************************
 * Function:     CntrlAutonOpenLoopTimer
 *
 * Description:  Control actuators that don't have feedback with an open loop timer.
 *
 ******************************************************************************/
bool CntrlAutonOpenLoopTimer(T_Actuator       L_CntrlActuator,
                             double           L_AutonTarget)
  {
  bool        L_ControlComplete = false;

  if ((L_CntrlActuator == E_ActuatorArmAngDwn) ||
      (L_CntrlActuator == E_ActuatorArmAngUp))
    {
    V_IntakeArmTimer += C_ExeTime;

    if (V_IntakeArmTimer < L_AutonTarget)
      {
      if (L_CntrlActuator == E_ActuatorArmAngDwn)
        {
        V_RobotUserCmndPct[E_RobotUserCmndIntakeArmAng] = K_AutonIntakeAngleCmnd;
        }
      else
        {
        V_RobotUserCmndPct[E_RobotUserCmndIntakeArmAng] = -K_AutonIntakeAngleCmnd;
        }
      }
    else
      {
      V_IntakeArmTimer = L_AutonTarget;

      V_RobotUserCmndPct[E_RobotUserCmndIntakeArmAng] = 0.0;

      L_ControlComplete = true;
      }

    }
  else if (L_CntrlActuator == E_ActuatorRollers)
    {
    V_RollerTimer += C_ExeTime;

    if (V_RollerTimer < L_AutonTarget)
      {
      V_RobotUserCmndPct[E_RobotUserCmndIntakeRoller] = -K_IntakeRollers; // Negative to eject cube
      }
    else
      {
      V_RollerTimer = L_AutonTarget;

      V_RobotUserCmndPct[E_RobotUserCmndIntakeRoller] = 0.0;

      L_ControlComplete = true;
      }
    }

  V_RobotMotorCmndPct[E_RobotMotorIntakeArmAng] = V_RobotUserCmndPct[E_RobotUserCmndIntakeArmAng];

  V_RobotMotorCmndPct[E_RobotMotorIntakeRoller] = V_RobotUserCmndPct[E_RobotUserCmndIntakeRoller];




  return (L_ControlComplete);
  }
