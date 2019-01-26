/*
  Control_LED.h

   Created on: Feb 20, 2018
       Author: 5561
 */
#ifndef SRC_ROBORIO2018_CONTROL_LED_H_
#define SRC_ROBORIO2018_CONTROL_LED_H_

void UpdateLED_Output(T_RoboState    L_RobotState,
                      bool           L_DriverOverride,
                      double         L_Winch,
                      T_RobotSide    L_TurnSignal,
                      DigitalOutput *L_LED_State0,
                      DigitalOutput *L_LED_State1,
                      DigitalOutput *L_LED_State2,
                      DigitalOutput *L_LED_State3);



#endif /* SRC_ROBORIO2018_CONTROL_LED_H_ */
