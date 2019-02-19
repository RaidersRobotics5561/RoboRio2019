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
