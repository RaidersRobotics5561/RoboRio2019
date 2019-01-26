/*
  VariableInit.hpp

   Created on: Feb 15, 2018
       Author: sdbig
 */
#ifndef SRC_ROBORIO2018_VARIABLEINIT_HPP_
#define SRC_ROBORIO2018_VARIABLEINIT_HPP_

#include "const.h"
#include "Vars.hpp"
#include "Calibrations.hpp"

extern void VariableInit(Preferences   *L_DriverPreferences,
                         Counter       *mCounter,
                         TalonSRX      *L_Talon0,
                         TalonSRX      *L_Talon1,
                         TalonSRX      *L_Talon2,
                         TalonSRX      *L_Talon3,
                         TalonSRX      *L_Talon4,
                         TalonSRX      *L_Talon5,
                         ADXRS450_Gyro *L_Gyro);

extern void AutonVariableInit(Preferences   *L_DriverPreferences,
                              Counter       *mCounter,
                              TalonSRX      *L_Talon0,
                              TalonSRX      *L_Talon1,
                              TalonSRX      *L_Talon2,
                              TalonSRX      *L_Talon3,
                              TalonSRX      *L_Talon4,
                              TalonSRX      *L_Talon5,
                              ADXRS450_Gyro *L_Gyro);

#endif /* SRC_ROBORIO2018_VARIABLEINIT_HPP_ */
