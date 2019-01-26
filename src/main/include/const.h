/*
 * const.h
 *
 *  Created on: Feb 1, 2018
 *      Author: wesat
 */

#ifndef SRC_CONST_H_
#define SRC_CONST_H_

#include "WPILib.h"
#include "Joystick.h"
#include <Spark.h>
#include "ctre/Phoenix.h"
#include "ADXRS450_Gyro.h"
#include "DigitalOutput.h"
#include "Enums.hpp"

const T_BotType C_BotType = E_BotComp; // Robot type

const double C_ExeTime = 0.01; // Execution rate of the Roborio controller

const double C_WheelPulsetoRev[E_RobotSideSz] =
    {  4100,  4100};

const double C_WheelDiameter[E_RobotSideSz] =
    { 6, 6};

const double C_PI = 3.14159265358979;

void Read_Sensors(TalonSRX  *L_DriveMortorCtrlLeft,
                  TalonSRX  *L_DriveMortorCtrlRight,
                  TalonSRX  *L_Intake,
                  TalonSRX  *L_Hook,
                  Counter   *L_ArmEncoder,
                  ADXRS450_Gyro *Gyro,
                  double    *L_ArmAngleDeg,
                  T_ArmCmnd  L_ArmCmndPrev,
                  T_ArmCmnd  L_ArmCmndPrevPrev,
                  Ultrasonic *L_UltraSonicSensorLeft,
                  Ultrasonic *L_UltraSonicSensorRight);
#endif /* SRC_CONST_H_ */
