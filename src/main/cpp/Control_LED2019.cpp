/*
  Control_LED.cpp

   Created on: Feb 11, 2018
       Author: Olivia

    This file controls the DIO pins which are used to communicate with the Arduino which ultimately controls the LED
    strip lights.  This file also takes a number of signals from the RoboRio main control and determines what mode the
    LED lights should be operating in.
 */

#include "const.h"
#include "Calibrations.hpp"
#include "Vars.hpp"




/******************************************************************************
 * Function:     UpdateLED_Output
 *
 * Description:  Update the commands sent out to the Arduino controlling the LEDs.
 *               The 16 modes are as follows:
 *
 *                * Chan0 * Chan1 * Chan2 * Chan3 *
 *                    0       0       0       0      * Mode0 * - Default no comm
 *                    0       0       1       1      * Mode1 * - Com, but disabled
 *                    0       1       0       0      * Mode2 * - Com, Rio and Pi
 *                    0       1       1       1      * Mode3 * - Sandstorm, Red Alliance
 *                    1       0       0       0      * Mode4 * - Sandstorm, Blue Alliance
 *                    1       0       1       1      * Mode5 * - Teleop, Red 
 *                    1       1       0       0      * Mode6 * - Teleop, Blue
 *                    1       1       1       1      * Mode7 * - User Override, Green
 *                    1       0       0       0      * Mode8 * - Raise on Stilts
 *                    1       0       0       1      * Mode9 * - On Stilts
 *                    1       0       1       0      * Mode10* - Both Stilts Up, Endgame
 *                    1       0       1       1      * Mode11* - Empty
 *                    1       1       0       0      * Mode12* - Empty
 *                    1       1       0       1      * Mode13* - Empty
 *                    1       1       1       0      * Mode14* - Empty
 *                    1       1       1       1      * Mode15* - Empty
 ******************************************************************************/
LED_Mode UpdateLED_Output(RoboState L_RobotState,
                          bool      L_DriverOverride,
                          double    L_Winch,
                          bool      *L_LED_CmndState[4])
  {
  bool L_Pin0            = false;
  bool L_Pin1            = false;
  bool L_Pin2            = false;
  bool L_Pin3            = false;
  LED_Mode L_LED_Mode    = LED_Mode0;
  double L_MatchTime     = DriverStation::GetInstance().GetMatchTime();

  DriverStation::Alliance L_AllianceColor;

  L_AllianceColor = DriverStation::GetInstance().GetAlliance();

  if (L_MatchTime < K_EndMatchWarningTime &&
      L_RobotState == C_Teleop &&
      V_LED_RainbowLatch == false &&
      fabs(L_Winch) > K_WinchOnThreshold)
    {
    /* Accumulate time that the winch has been on while in the end game time.  Once the winch has been on long enough,
     * trigger the final LED effect. */
    V_EndGameWinchTime += C_ExeTime;
    if (V_EndGameWinchTime >= K_LED_WinchOnTime)
      {
      V_LED_RainbowLatch = true;
      }
    }

  if (L_DriverOverride == true)
    {
    /* Allow the driver to always override the current LED mode. */
      L_LED_Mode = LED_Mode7;
    }
  else if ((L_RobotState == C_Auton || L_RobotState == C_Teleop) &&
           (L_AllianceColor != DriverStation::Alliance::kInvalid))
    {
      if (V_LED_RainbowLatch == true)
        {
          L_LED_Mode = LED_Mode11;
        }
      else if (L_AllianceColor == DriverStation::Alliance::kBlue)
        {
          if (/*L_MatchTime > K_EndMatchWarningTime ||*/ L_RobotState == C_Auton)
            {
              if (L_AutonState == C_Right)
            	  {
            	  L_LED_Mode = LED_Mode13;
            	  }
              else if (L_AutonState == C_Left)
              	  {
            	  L_LED_Mode = LED_Mode12;
              	  }
              else
              	  {
            	  L_LED_Mode = LED_Mode3;
              	  }
            }
          else if (L_MatchTime <= K_ENdMatchWarningTime)
          {
        	  L_LED_Mode = LED_Mode8;
          }
          else
            {
              L_LED_Mode = LED_Mode6;
            }
        }
      else if (L_AllianceColor == DriverStation::Alliance::kRed)
        {
          if (/*L_MatchTime > K_EndMatchWarningTime ||*/ L_RobotState == C_Auton)
            {
        	  if (L_AutonState == C_Right)
        	  	  {
        		  L_LED_Mode = LED_Mode13;
        	  	  }
        	  else if (L_AutonState == C_Left)
        	  	  {
        		  L_LED_Mode = LED_Mode12;
        	  	  }
        	  else
        	  	  {
        		  L_LED_Mode = LED_Mode7;
        	  	  }
        }
          else if (L_MatchTime <= K_EndMatchWarningTime)
          {
        	  L_LED_Mode = LED_Mode8;
          }
          else
            {
              L_LED_Mode = LED_Mode5;
            }
        }
    }
  else if (L_RobotState == C_Disabled)
    {
      L_LED_Mode = LED_Mode0;
    }
  else if (L_RobotState == C_Test)
    {
      L_LED_Mode = LED_Mode2;
    }

  switch (L_LED_Mode)
    {
      case LED_Mode1:
        L_Pin0 = false; L_Pin1 = false; L_Pin2 = false; L_Pin3 = true;
        break;

      case LED_Mode2:
        L_Pin0 = false; L_Pin1 = false; L_Pin2 = true; L_Pin3 = false;
        break;

      case LED_Mode3:
        L_Pin0 = false; L_Pin1 = false; L_Pin2 = true; L_Pin3 = true;
        break;

      case LED_Mode4:
        L_Pin0 = false; L_Pin1 = true; L_Pin2 = false; L_Pin3 = false;
        break;

      case LED_Mode5:
        L_Pin0 = false; L_Pin1 = true; L_Pin2 = false; L_Pin3 = true;
        break;

      case LED_Mode6:
        L_Pin0 = false; L_Pin1 = true; L_Pin2 = true; L_Pin3 = false;
        break;

      case LED_Mode7:
        L_Pin0 = false; L_Pin1 = true; L_Pin2 = true; L_Pin3 = true;
        break;

      case LED_Mode8:
        L_Pin0 = true;  L_Pin1 = false; L_Pin2 = false; L_Pin3 = false;
        break;

      case LED_Mode9:
        L_Pin0 = true; L_Pin1 = false; L_Pin2 = false; L_Pin3 = true;
        break;

      case LED_Mode10:
        L_Pin0 = true; L_Pin1 = false; L_Pin2 = true; L_Pin3 = false;
        break;

      case LED_Mode11:
        L_Pin0 = true; L_Pin1 = false; L_Pin2 = true; L_Pin3 = true;
        break;

      case LED_Mode12:
        L_Pin0 = true; L_Pin1 = true; L_Pin2 = false; L_Pin3 = false;
        break;

      case LED_Mode13:
        L_Pin0 = true; L_Pin1 = true; L_Pin2 = false; L_Pin3 = true;
        break;

      case LED_Mode14:
        L_Pin0 = true; L_Pin1 = true; L_Pin2 = true; L_Pin3 = false;
        break;

      case LED_Mode15:
        L_Pin0 = true; L_Pin1 = true; L_Pin2 = true; L_Pin3 = true;
        break;

      case LED_Mode0:
      default:
        L_Pin0 = false; L_Pin1 = false; L_Pin2 = false; L_Pin3 = false;
        break;
  }


  // Output to the DIO pins:
  *L_LED_CmndState[0]= L_Pin0;
  *L_LED_CmndState[1]= L_Pin1;
  *L_LED_CmndState[2]= L_Pin2;
  *L_LED_CmndState[3]= L_Pin3;
//  V_LED_State[0].Set(L_Pin0);
//  V_LED_State[1].Set(L_Pin1);
//  V_LED_State[2].Set(L_Pin2);
//  V_LED_State[3].Set(L_Pin3);

  return (L_LED_Mode);
  }
