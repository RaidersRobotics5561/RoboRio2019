/*
  enums.h

   Created on: Feb 14, 2019
       Author: 5561
 */

typedef enum T_RobotSide {
  E_RobotSideLeft, E_RobotSideRight, E_RobotSideSz
} T_RobotSide;

typedef enum T_RobotLiftSide {
    E_RobotLiftForward, E_RobotLiftBack, E_RobotLiftSz
} T_RobotLiftSide;


typedef enum T_RobotShimmyRight {
    E_RobotShimmyRight_LeftBackwards,
    E_RobotShimmyRight_RightBackwards,
    E_RobotShimmyRight_LeftForward,
    E_RobotShimmyRight_RightForward,
    E_RobotShimmyRight_ShimmySz
} T_RobotShimmyRight;

typedef enum T_RobotShimmyLeft {
    E_RobotShimmyLeft_RightBackwards,
    E_RobotShimmyLeft_LeftBackwards,
    E_RobotShimmyLeft_RightForward,
    E_RobotShimmyLeft_LeftForward,
    E_RobotShimmyLeft_ShimmySz
} T_RobotShimmyLeft;

 typedef enum T_RoboState
 {
   E_Init,
   E_Disabled,
   E_DisabledButReady,
   E_Teleop,
   E_AutonSandStorm1,
   E_AutonSandStorm2,
   E_AutonEndGame1,
   E_AutonEndGame2,
   E_AutonEndGame3,
   E_AutonEndGame4,
   E_AutonEndGame5,
   E_Test,
   E_Null
 }T_RoboState;

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