/*
  Control_PID.hpp

   Created on: Feb 14, 2018
       Author: 5561
 */
#ifndef SRC_ROBORIO2018_CONTROL_PID_HPP_
#define SRC_ROBORIO2018_CONTROL_PID_HPP_

extern double Control_PID(double  L_DesiredSpeed,
                          double  L_CurrentSpeed,
                          double *L_ErrorPrev,
                          double *L_IntegralPrev,
                          double  L_ProportionalGx,
                          double  L_IntegralGx,
                          double  L_DerivativeGx,
                          double  L_ProportionalUpperLimit,
                          double  L_ProportionalLowerLimit,
                          double  L_IntegralUpperLimit,
                          double  L_IntegralLowerLimit,
                          double  L_DerivativeUpperLimit,
                          double  L_DerivativeLowerLimit,
                          double  L_OutputUpperLimit,
                          double  L_OutputLowerLimit);

extern double LukeStoppers(double L_DesiredSpeed,
                           double L_CurrentSpeed,
                           double L_RampRate);

extern double DesiredSpeed(double L_JoystickAxis,
                           double L_CurrentSpeed);

extern double DesiredLiftHeight(double L_JoystickAxis,
                                double L_DesiredLiftHeightPrev,
                                double L_MaxHeight);

extern double LiftCmdDisable(double LiftHight,
                             double CommandedHight,
                             double MinHight,
                             double L_MotorCmnd);

#endif /* SRC_ROBORIO2018_CONTROL_PID_HPP_ */
