/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {

  int K_TimeoutMs = 10;

  _talon0->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
  
  _talon0->SetSensorPhase(true);

  _talon0->ConfigNominalOutputForward(0, K_TimeoutMs);
  _talon0->ConfigNominalOutputReverse(0, K_TimeoutMs);
  _talon0->ConfigPeakOutputForward(1, K_TimeoutMs);
  _talon0->ConfigPeakOutputReverse(-1, K_TimeoutMs);
  _talon0->SetSelectedSensorPosition(0, 0, K_TimeoutMs);

}

void Robot::TeleopPeriodic() {
  
  while( IsOperatorControl() && IsEnabled() ){

    _talon0->Set(ControlMode::PercentOutput, 0.25);

    frc::Wait(0.01);
  }

}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
