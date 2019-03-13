#pragma once

#include <frc/IterativeRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <frc/Ultrasonic.h>
#include <frc/Joystick.h>
#include <frc/Spark.h>

#include "enums.h"
#include "const.h"
#include "Control_Pid.h"
#include "Control_Auton.h"
#include <frc/DigitalOutput.h>

class Robot : public frc::IterativeRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  double Lift_Pos[E_RobotLiftSz];
 
  bool IsAuton    = false;
  bool IsAuton2   = false;
  bool IsAutonDwn = false;

  double DesiredPos_Backward = 0; 
  double DesiredPos_Forward = 0; 

  double ErrPrev_Lift[E_RobotLiftSz] = {0,0};
  
  double IntPrev_Lift[E_RobotLiftSz] = {0,0};

  TalonSRX * _talon1 = new TalonSRX(1);
  TalonSRX * _talon2 = new TalonSRX(2);
  TalonSRX * _talon3 = new TalonSRX(3);
  TalonSRX * _talon4 = new TalonSRX(4);

  TalonSRX * _talon6 = new TalonSRX(6);
  TalonSRX * _talon5 = new TalonSRX(5);

  Spark *_spark1 = new Spark(0);
  Spark *_spark2 = new Spark(1);
  Spark *_spark3 = new Spark(2);

  Joystick *_joy1 = new Joystick(0);
  Joystick *_joy2 = new Joystick(1);

  Ultrasonic *_UltraFront;
  Ultrasonic *_UltraBack;

	DigitalOutput *V_LED_State0 = new DigitalOutput(6);
	DigitalOutput *V_LED_State1 = new DigitalOutput(7);
	DigitalOutput *V_LED_State2 = new DigitalOutput(8);
	DigitalOutput *V_LED_State3 = new DigitalOutput(9);
};
