/*
  Control_Auton.hpp

   Created on: Feb 25, 2018
       Author: sdbig
 */
#ifndef SRC_ROBORIO2018_CONTROL_AUTON_HPP_
#define SRC_ROBORIO2018_CONTROL_AUTON_HPP_


extern T_AutonOpt DtrmnAutonOption(T_RobotSide       L_AutonTargetSwitch,
                                   T_RobotSide       L_AutonTargetScale,
                                   T_AutonStartPos   L_AutonStartPos,
                                   T_AutonPreference L_AutonPreference);

extern bool DtrmnActuatorComplete(double     L_CntrlVal,
                                  double     L_MeasuredVal,
                                  double     L_Deadband,
                                  double    *L_DebounceTimer,
                                  double     L_DebounceThreshold);

extern double CntrlAutonDesiredSpeed(double            L_TotalPlannedTravel,
                                     double            L_MeasuredTravel,
                                     double            L_DesiredSpeedPrev,
                                     double            L_RampDownTravel,
                                     double            L_K_MinSpeed,
                                     double            L_K_MaxSpeed,
                                     double            L_K_SpeedRamp,
                                     double            L_DebounceTime,
                                     bool              L_TargetMet);

extern bool CntrlAutonDrive(T_Actuator       L_CntrlActuator,
                            double           L_AutonTarget);


extern bool CntrlAutonLift(T_Actuator       L_CntrlActuator,
                           double           L_AutonTarget);

extern bool CntrlAutonOpenLoopTimer(T_Actuator       L_CntrlActuator,
                                    double           L_AutonTarget);

#endif /* SRC_ROBORIO2018_CONTROL_AUTON_HPP_ */
