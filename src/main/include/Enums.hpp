/*
  Enums.hpp

   Created on: Feb 14, 2018
       Author: 5561
 */
#ifndef SRC_ROBORIO2018_ENUMS_HPP_
#define SRC_ROBORIO2018_ENUMS_HPP_

 typedef enum Blinker
 {
   C_BlinkR,
   C_BlinkL,
   C_BlinkSz
 }Blinker;

 typedef enum T_DriveMode
 {
	 E_TankDrive,
	 E_ArcadeDrive,
	 E_ArdadeDriveSimple
 } T_DriveMode;

 typedef enum LED_Color
 {
   LED_Color_Red,
   LED_Color_Blue,
   LED_Color_Green,
   LED_Color_White,
   LED_Color_Purple,
   LED_Color_Yellow,
   LED_Color_Pink,
   LED_Color_Orange,
   LED_Color_Rainbow, // This is meant to indicate when a random mixture of colors are desired
   LED_Color_Multi,   // This is meant to indicate when it is desired to have the colors cycle through all of the avaiable colors above rainbow
   LED_Color_Black    // This is more of an "off mode", must remain at end of enum
 } LED_Color;

 typedef enum LED_Mode
 {
   LED_Mode0,
   LED_Mode1,
   LED_Mode2,
   LED_Mode3,
   LED_Mode4,
   LED_Mode5,
   LED_Mode6,
   LED_Mode7,
   LED_Mode8,
   LED_Mode9,
   LED_Mode10,
   LED_Mode11,
   LED_Mode12,
   LED_Mode13,
   LED_Mode14,
   LED_Mode15,
   LED_ModeSz
 } LED_Mode;

 typedef enum T_RoboState
 {
   E_Disabled,
   E_Auton,
   E_AutonComplete,
   E_Teleop,
   E_Test,
   E_Null
 }T_RoboState;



 typedef enum Actuators
 {
   C_DriveMotorL,
   C_DriveMotorR,
   C_IntakeArmAngle,
   C_IntakeRollers,
   C_IntakeLift,
   C_Winch,
   C_Hook,
   C_ActuatorsSz
 }Actuators;

 typedef enum T_BotType {
   E_BotComp, E_BotPractice, E_BotTestBoard, E_BotSz
 } T_BotType;

typedef enum T_RobotSide {
  E_RobotSideLeft, E_RobotSideRight, E_RobotSideSz
} T_RobotSide;

typedef enum T_RobotUserCmnd {
  E_RobotUserCmndLeftWheel,
  E_RobotUserCmndRightWheel,
  E_RobotUserCmndLift,
  E_RobotUserCmndIntakeArmAng,
  E_RobotUserCmndIntakeRoller,
  E_RobotUserCmndHook,
  E_RobotUserCmndWinch,
  E_RobotUserCmndSz
} T_RobotUserCmnd;

typedef enum T_RobotMotor {
  E_RobotMotorLeftWheel,
  E_RobotMotorRightWheel,
  E_RobotMotorLift,
  E_RobotMotorIntakeArmAng,
  E_RobotMotorIntakeRoller,
  E_RobotMotorHook,
  E_RobotMotorWinch,
  E_RobotMotorSz
} T_RobotMotor;

typedef enum T_PID {
  E_PID_Proportional, E_PID_Integral, E_PID_Derivative, E_PID_Sz
} T_PID;

typedef enum T_IntergalLimit {
  E_IntergalUpperLimit, E_IntergalLowerLimit, E_IntergalLimitSz
} T_IntergalLimit;

typedef enum T_ArmCmnd
{
  E_ArmCmndOff,
  E_ArmCmndUp,
  E_ArmCmndDwn,
  E_ArmCmndSz
} T_ArmCmnd;

typedef enum T_AutonStartPos
  {
    E_AutonStartPosLeft,
    E_AutonStartPosMiddle,
    E_AutonStartPosRight,
    E_AutonStartPosDefault
  } T_AutonStartPos;

typedef enum T_AutonPreference
  {
    E_AutonPreferenceSwitch,
    E_AutonPreferenceScale,
    E_AutonPreferenceSz
  } T_AutonPreference;

typedef enum T_AutonOpt
  {
    E_AutonOpt0,
    E_AutonOpt1,
    E_AutonOpt2,
    E_AutonOpt3,
    E_AutonOpt4,
    E_AutonOpt5,
    E_AutonOpt6,
    E_AutonOptSz
  } T_AutonOpt;

typedef enum T_Actuator
  {
    E_ActuatorNone,
    E_ActuatorDriveEncoder,
    E_ActuatorDriveUltraSonic,
    E_ActuatorRotate,
    E_ActuatorLift,
    E_ActuatorArmAngDwn,
    E_ActuatorArmAngUp,
    E_ActuatorRollers,
    E_ActuatorSz
  } T_Actuator;

  typedef enum T_AutonStep
    {
      E_AutonStep0,
      E_AutonStep1,
      E_AutonStep2,
      E_AutonStep3,
      E_AutonStep4,
      E_AutonStep5,
      E_AutonStep6,
      E_AutonStep7,
      E_AutonStep8,
      E_AutonStep9,
      E_AutonStep10,
      E_AutonStep11,
      E_AutonStep12,
      E_AutonStep13,
      E_AutonStep14,
      E_AutonStepSz
    } T_AutonStep;

  typedef enum T_AutonCntrlType
    {
      E_AutonCntrlPrimary,
      E_AutonCntrlSecondary,
      E_AutonCntrlSz
    } T_AutonCntrlType;

typedef struct AutonControlPlan
  {
  T_Actuator PrimaryActuator[E_AutonStepSz];
  double     PrimaryCmndValue[E_AutonStepSz];
  T_Actuator SecondaryActuator[E_AutonStepSz];
  double     SecondaryCmndValue[E_AutonStepSz];
  } AutonControlPlan;

#endif /* SRC_ROBORIO2018_ENUMS_HPP_ */
