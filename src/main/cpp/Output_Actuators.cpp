/*
  Output_Actuators.cpp

   Created on: Feb 19, 2018
       Author: 5561
 */

#include "Const.h"
#include "Vars.hpp"

/******************************************************************************
 * Function:     UpdateActuatorCmnds
 *
 * Description:  Update the commands sent out from the RoboRIO to the motor
 *               controllers and LEDs.
 ******************************************************************************/
void UpdateActuatorCmnds(TalonSRX  *L_DriveMortorCtrlLeftFront,
                         TalonSRX  *L_DriveMortorCtrlLeftRear,
                         TalonSRX  *L_DriveMortorCtrlRightFront,
                         TalonSRX  *L_DriveMortorCtrlRightRear,
                         TalonSRX  *L_Intake,
                         TalonSRX  *L_Hook,
                         Spark     *L_Spark0,
                         Spark     *L_Spark1,
                         Talon     *Talon_PWM0,
                         Talon     *Talon_PWM1,
                         Talon     *Talon_PWM2,
                         double    *L_RobotMotorCmndPct)
  {
  L_DriveMortorCtrlLeftFront->Set(ControlMode::PercentOutput,L_RobotMotorCmndPct[E_RobotMotorLeftWheel]);
  L_DriveMortorCtrlLeftRear->Set(ControlMode::PercentOutput,L_RobotMotorCmndPct[E_RobotMotorLeftWheel]);
  L_DriveMortorCtrlRightFront->Set(ControlMode::PercentOutput,(L_RobotMotorCmndPct[E_RobotMotorRightWheel] * -1));
  L_DriveMortorCtrlRightRear->Set(ControlMode::PercentOutput,(L_RobotMotorCmndPct[E_RobotMotorRightWheel] * -1));

  L_Intake->Set(ControlMode::PercentOutput, (-L_RobotMotorCmndPct[E_RobotMotorLift]));

  L_Hook->Set(ControlMode::PercentOutput, (-L_RobotMotorCmndPct[E_RobotMotorHook]));

  L_Spark0->Set(L_RobotMotorCmndPct[E_RobotMotorIntakeArmAng]); // Intake angle
  L_Spark1->Set(L_RobotMotorCmndPct[E_RobotMotorIntakeArmAng]); // Intake angle

  Talon_PWM0->Set(L_RobotMotorCmndPct[E_RobotMotorIntakeRoller]); // Intake roller
  Talon_PWM1->Set(-L_RobotMotorCmndPct[E_RobotMotorIntakeRoller]); // Intake roller
  Talon_PWM2->Set(L_RobotMotorCmndPct[E_RobotMotorWinch]);
  }
