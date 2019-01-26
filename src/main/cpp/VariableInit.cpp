/*
  VariableInit.cpp

   Created on: Feb 13, 2018
       Author: 5561
 */

#include "VariableInit.hpp"


/******************************************************************************
 * Function:     VariableInit
 *
 * Description:  Initialize the necessary variables when the robot switches
 *               modes (i.e. from auton to teleop).
 ******************************************************************************/
void VariableInit(Preferences   *L_DriverPreferences,
                  Counter       *mCounter,
                  TalonSRX      *L_Talon0,
                  TalonSRX      *L_Talon1,
                  TalonSRX      *L_Talon2,
                  TalonSRX      *L_Talon3,
                  TalonSRX      *L_Talon4,
                  TalonSRX      *L_Talon5,
                  ADXRS450_Gyro *L_Gyro)
  {
  T_RobotSide  L_RobotSide;
  T_RobotMotor L_RobotMotor;
  int          L_Index;

  //Reset Sensor Position
  L_Talon0->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
  L_Talon1->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
  L_Talon2->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
  L_Talon3->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
  L_Talon4->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
  L_Talon5->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);


  mCounter->Reset();

  V_WheelProportionalGain[E_RobotSideLeft] = L_DriverPreferences->GetDouble("P_L", K_WheelSpeedPID_Gain[E_RobotSideLeft][E_PID_Proportional]);
  V_WheelIntegralGain[E_RobotSideLeft] = L_DriverPreferences->GetDouble("I_L", K_WheelSpeedPID_Gain[E_RobotSideLeft][E_PID_Integral]);
  V_WheelDerivativeGain[E_RobotSideLeft] = L_DriverPreferences->GetDouble("D_L", K_WheelSpeedPID_Gain[E_RobotSideLeft][E_PID_Derivative]);
  V_WheelProportionalGain[E_RobotSideRight] = L_DriverPreferences->GetDouble("P_R", K_WheelSpeedPID_Gain[E_RobotSideRight][E_PID_Proportional]);
  V_WheelIntegralGain[E_RobotSideRight] = L_DriverPreferences->GetDouble("I_R", K_WheelSpeedPID_Gain[E_RobotSideRight][E_PID_Integral]);
  V_WheelDerivativeGain[E_RobotSideRight] = L_DriverPreferences->GetDouble("D_R", K_WheelSpeedPID_Gain[E_RobotSideRight][E_PID_Derivative]);
  V_WheelSpeedLagFiltGain[E_RobotSideRight] = L_DriverPreferences->GetDouble("Lag_R", K_WheelSpeedLagFilterGain[E_RobotSideRight]);
  V_WheelSpeedLagFiltGain[E_RobotSideLeft] = L_DriverPreferences->GetDouble("Lag_L", K_WheelSpeedLagFilterGain[E_RobotSideLeft]);

  V_IntakeArmPulseToRev[E_ArmCmndOff] =  L_DriverPreferences->GetDouble("ArmPulseOff", K_IntakeArmPulseToRev[E_ArmCmndOff]);
  V_IntakeArmPulseToRev[E_ArmCmndUp] =  L_DriverPreferences->GetDouble("ArmPulseUp", K_IntakeArmPulseToRev[E_ArmCmndUp]);
  V_IntakeArmPulseToRev[E_ArmCmndDwn] =  L_DriverPreferences->GetDouble("ArmPulseDwn", K_IntakeArmPulseToRev[E_ArmCmndDwn]);

  V_IntakePID_Gain[E_PID_Proportional] = L_DriverPreferences->GetDouble("P_Lift", K_Intake_PID_Gain[E_PID_Derivative]);
  V_IntakePID_Gain[E_PID_Integral] = L_DriverPreferences->GetDouble("I_Lift", K_Intake_PID_Gain[E_PID_Derivative]);
  V_IntakePID_Gain[E_PID_Derivative] = L_DriverPreferences->GetDouble("D_Lift", K_Intake_PID_Gain[E_PID_Derivative]);

  V_IntakePosition = 0.0;
  V_IntakePositionPrev = 0.0;
  V_IntakeLiftHeightDesired = 0.0;

  V_HookPositionErrorPrev = 0.0;
  V_HookPositionErrorIntegral = 0.0;
  V_HookLiftHeightDesired = 0.0;
  V_HookPID_Gain[E_PID_Proportional] = L_DriverPreferences->GetDouble("P_Hook", K_Hook_PID_Gain[E_PID_Derivative]);
  V_HookPID_Gain[E_PID_Integral] = L_DriverPreferences->GetDouble("I_Hook", K_Hook_PID_Gain[E_PID_Derivative]);
  V_HookPID_Gain[E_PID_Derivative] = L_DriverPreferences->GetDouble("D_Hook", K_Hook_PID_Gain[E_PID_Derivative]);
  V_HookPosition = 0.0;
  V_RotateGain = L_DriverPreferences->GetDouble("RotateGain", K_RotateGain);

  V_AutonRotateDebounceTimer   = 0.0;
  V_AutonWheelDebounceTimer[E_RobotSideLeft]  = 0.0;
  V_AutonWheelDebounceTimer[E_RobotSideRight] = 0.0;
  V_AutonIntakeLiftDebounceTimer = 0.0;
  V_LukeStopperRamp = L_DriverPreferences->GetDouble("LukeStopper", K_LukeStopperRamp);

  DriveMode = E_ArcadeDrive;

  V_ArmAngleDeg = 0.0;
  for (L_RobotSide = E_RobotSideLeft;
       L_RobotSide < E_RobotSideSz;
       L_RobotSide = T_RobotSide(int(L_RobotSide) + 1))
    {
    V_WheelSpeedErrorPrev[L_RobotSide] = 0.0;
    V_WheelSpeedErrorIntegral[L_RobotSide] = 0.0;
    V_WheelRPM_Raw[L_RobotSide] = 0.0;
    V_WheelRPM_Filt[L_RobotSide] = 0.0;
    V_WheelRPM_FiltPrev[L_RobotSide] = 0.0;
    V_WheelRPM_Desired[L_RobotSide] = 0.0;
    V_UltraSonicDistance[L_RobotSide] = 0.0;
    }

  /* Reset all commands to zero: */
  for (L_RobotMotor = E_RobotMotorLeftWheel;
      L_RobotMotor < E_RobotMotorSz;
      L_RobotMotor = T_RobotMotor(int(L_RobotMotor) + 1))
    {
    V_RobotMotorCmndPct[L_RobotMotor] = 0.0;
    }

  for (L_Index = 0; L_Index < 3; L_Index++)
    {
    V_AutonTargetSide[L_Index] = E_RobotSideRight;
    }

  V_RollerTimer = 0.0;
  V_IntakeArmTimer = 0.0;

  V_LED_RainbowLatch = false;

  V_GyroAngleOffset = L_Gyro->GetAngle();
  }


/******************************************************************************
 * Function:     AutonVariableInit
 *
 * Description:  This is a reduced list of variables to reset when auton is
 *               running.
 ******************************************************************************/
void AutonVariableInit(Preferences   *L_DriverPreferences,
                       Counter       *mCounter,
                       TalonSRX      *L_Talon0,
                       TalonSRX      *L_Talon1,
                       TalonSRX      *L_Talon2,
                       TalonSRX      *L_Talon3,
                       TalonSRX      *L_Talon4,
                       TalonSRX      *L_Talon5,
                       ADXRS450_Gyro *L_Gyro)
  {
  T_RobotSide  L_RobotSide;
  T_RobotMotor L_RobotMotor;
  int          L_Index;
  double       L_LiftHeightCmndPrev = 0.0;
  double       L_HookHeightCmndPrev = 0.0;


  //Reset Sensor Position
  L_Talon0->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
  L_Talon1->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
  L_Talon2->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
  L_Talon3->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);

  mCounter->Reset();

  V_AutonRotateDebounceTimer   = 0.0;
  V_AutonWheelDebounceTimer[E_RobotSideLeft]  = 0.0;
  V_AutonWheelDebounceTimer[E_RobotSideRight] = 0.0;
  V_AutonIntakeLiftDebounceTimer = 0.0;

  for (L_RobotSide = E_RobotSideLeft;
       L_RobotSide < E_RobotSideSz;
       L_RobotSide = T_RobotSide(int(L_RobotSide) + 1))
    {
    V_WheelSpeedErrorPrev[L_RobotSide]     = 0.0;
    V_WheelSpeedErrorIntegral[L_RobotSide] = 0.0;
    V_WheelRPM_Raw[L_RobotSide]            = 0.0;
    V_WheelRPM_Filt[L_RobotSide]           = 0.0;
    V_WheelRPM_FiltPrev[L_RobotSide]       = 0.0;
    V_WheelRPM_Desired[L_RobotSide]        = 0.0;
    V_UltraSonicDistance[L_RobotSide]      = 0.0;
    }

  /* Reset all commands to zero (with the exception of the lift mechanism and hook): */
  L_LiftHeightCmndPrev = V_RobotMotorCmndPct[E_RobotMotorLift];
  L_HookHeightCmndPrev = V_RobotMotorCmndPct[E_RobotMotorHook];

  for (L_RobotMotor = E_RobotMotorLeftWheel;
      L_RobotMotor < E_RobotMotorSz;
      L_RobotMotor = T_RobotMotor(int(L_RobotMotor) + 1))
    {
    V_RobotMotorCmndPct[L_RobotMotor] = 0.0;
    }

  V_RobotMotorCmndPct[E_RobotMotorLift] = L_LiftHeightCmndPrev;
  V_RobotMotorCmndPct[E_RobotMotorHook] = L_HookHeightCmndPrev;

  V_RollerTimer = 0.0;
  V_IntakeArmTimer = 0.0;

  V_GyroAngleOffset = L_Gyro->GetAngle();
  }
