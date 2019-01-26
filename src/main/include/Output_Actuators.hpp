/*
  Output_Actuators.hpp

   Created on: Feb 19, 2018
       Author: sdbig
 */
#ifndef SRC_ROBORIO2018_OUTPUT_ACTUATORS_HPP_
#define SRC_ROBORIO2018_OUTPUT_ACTUATORS_HPP_

#include "const.h"

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
                         double    *L_RobotMotorCmndPct);

#endif /* SRC_ROBORIO2018_OUTPUT_ACTUATORS_HPP_ */
