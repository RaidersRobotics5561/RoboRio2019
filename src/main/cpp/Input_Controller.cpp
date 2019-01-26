/*
  Input_Controller.cpp

   Created on: Feb 16, 2018
       Author: 5561
 */

#include "const.h"
#include "Calibrations.hpp"
#include "Vars.hpp"
#include "SignalFilter.hpp"

/******************************************************************************
 * Function:     DtrmnControllerMapping
 *
 * Description:  Map the controller to the desired inputs/commands.
 ******************************************************************************/
void DtrmnControllerMapping(Joystick *L_Joystick1,
                            Joystick *L_Joystick2)
  {
  double L_IntakeRollers = 0.0;
  double Winch = 0.0;

    if (L_Joystick1->GetPOV() == 0)
      {
      DriveMode = E_TankDrive;
      }
    else if (L_Joystick1->GetPOV() == 90)
      {
      DriveMode = E_ArcadeDrive;
      }
    else if (L_Joystick1->GetPOV() == 270)
      {
      DriveMode = E_ArdadeDriveSimple;
      }

    //Code for different DriveModes
    switch (DriveMode)
      {
      case E_TankDrive: //Tank Drive
        V_RobotUserCmndPct[E_RobotUserCmndLeftWheel]  = -L_Joystick1->GetRawAxis(1);
        V_RobotUserCmndPct[E_RobotUserCmndRightWheel] = -L_Joystick1->GetRawAxis(5);
      break;

      case E_ArcadeDrive: //Arcade Drive
        V_RobotUserCmndPct[E_RobotUserCmndLeftWheel]  = -(L_Joystick1->GetRawAxis(1) - (L_Joystick1->GetRawAxis(4) * V_RotateGain));
        V_RobotUserCmndPct[E_RobotUserCmndRightWheel] = -(L_Joystick1->GetRawAxis(1) + (L_Joystick1->GetRawAxis(4) * V_RotateGain));
      break;

      case E_ArdadeDriveSimple: //Arcade Drive Simple
        V_RobotUserCmndPct[E_RobotUserCmndLeftWheel] = -(L_Joystick1->GetRawAxis(1) - (L_Joystick1->GetRawAxis(0) * V_RotateGain));
        V_RobotUserCmndPct[E_RobotUserCmndRightWheel] = -(L_Joystick1->GetRawAxis(1) + (L_Joystick1->GetRawAxis(0) * V_RotateGain));
      break;
    }

 // Combination of right and left trigger
    if(L_Joystick2->GetRawButton(5)){
      V_RobotUserCmndPct[E_RobotUserCmndHook] = -1;
    } else if(L_Joystick2->GetRawButton(6)){
      V_RobotUserCmndPct[E_RobotUserCmndHook] =  1;
    } else {
  V_RobotUserCmndPct[E_RobotUserCmndHook] = 0;
    }

//  if (L_Joystick2->GetRawButton(1) == true)
//    {
//    L_IntakeRollers = K_IntakeRollers;
//    }
//  else if (L_Joystick2->GetRawButton(2) == true)
//    {
//    L_IntakeRollers = -K_IntakeRollers;
//    }



  if(L_Joystick2->GetRawButton(4)){
   Winch = K_Winch;
  }
  else if (L_Joystick2->GetRawButton(3))
    {
    Winch = -K_Winch;
    }

  L_IntakeRollers = DeadBand(L_Joystick2->GetRawAxis(5),
                             -K_JoystickAnalogDeadband,
                              K_JoystickAnalogDeadband);

  V_RobotUserCmndPct[E_RobotUserCmndIntakeArmAng] = DeadBand(L_Joystick2->GetRawAxis(1),
                                                             -K_JoystickAnalogDeadband,
                                                              K_JoystickAnalogDeadband);

  V_RobotUserCmndPct[E_RobotUserCmndLift] = DeadBand((L_Joystick2->GetRawAxis(3) - (L_Joystick2->GetRawAxis(2)*K_LiftSpeedDropGain)),
                                                     -K_JoystickAnalogDeadband,
                                                      K_JoystickAnalogDeadband);

  if ((V_RobotUserCmndPct[E_RobotUserCmndIntakeArmAng] > 0.0) ||
      (V_RobotUserCmndPct[E_RobotUserCmndLift] > 0.0))
    {
    L_IntakeRollers += K_IntakeRollerAutoPullIn;

    if (L_IntakeRollers > 1.0)
      {
      L_IntakeRollers = 1.0;
      }
    }

  V_RobotUserCmndPct[E_RobotUserCmndIntakeRoller] = L_IntakeRollers;

  V_RobotUserCmndPct[E_RobotUserCmndWinch] =  Winch;
  }
