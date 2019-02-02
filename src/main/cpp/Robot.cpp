/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

using namespace frc;

void Robot::RobotInit() {
  	_talon6->ConfigSelectedFeedbackSensor(
			FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
    
    //Back Lift
    _talon6->SetSensorPhase(true);
    _talon6->ConfigNominalOutputForward(0, K_TimeoutMs);
    _talon6->ConfigNominalOutputReverse(0, K_TimeoutMs);
    _talon6->ConfigPeakOutputForward(1, K_TimeoutMs);
    _talon6->ConfigPeakOutputReverse(-1, K_TimeoutMs);
    _talon6->SetSelectedSensorPosition(0, 0, K_TimeoutMs);
    

    //Forward Lift
    _talon5->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
    _talon5->SetSensorPhase(true);
    _talon5->ConfigNominalOutputForward(0, K_TimeoutMs);
    _talon5->ConfigNominalOutputReverse(0, K_TimeoutMs);
    _talon5->ConfigPeakOutputForward(1, K_TimeoutMs);
    _talon5->ConfigPeakOutputReverse(-1, K_TimeoutMs);
    _talon5->SetSelectedSensorPosition(0, 0, K_TimeoutMs);

    //
    _talon1->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
    _talon1->SetSensorPhase(true);
    _talon1->ConfigNominalOutputForward(0, K_TimeoutMs);
    _talon1->ConfigNominalOutputReverse(0, K_TimeoutMs);
    _talon1->ConfigPeakOutputForward(1, K_TimeoutMs);
    _talon1->ConfigPeakOutputReverse(-1, K_TimeoutMs);
    _talon1->SetSelectedSensorPosition(0, 0, K_TimeoutMs);

    _talon2->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
    _talon2->SetSensorPhase(true);
    _talon2->ConfigNominalOutputForward(0, K_TimeoutMs);
    _talon2->ConfigNominalOutputReverse(0, K_TimeoutMs);
    _talon2->ConfigPeakOutputForward(1, K_TimeoutMs);
    _talon2->ConfigPeakOutputReverse(-1, K_TimeoutMs);
    _talon2->SetSelectedSensorPosition(0, 0, K_TimeoutMs);

    _talon3->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
    _talon3->SetSensorPhase(true);
    _talon3->ConfigNominalOutputForward(0, K_TimeoutMs);
    _talon3->ConfigNominalOutputReverse(0, K_TimeoutMs);
    _talon3->ConfigPeakOutputForward(1, K_TimeoutMs);
    _talon3->ConfigPeakOutputReverse(-1, K_TimeoutMs);
    _talon3->SetSelectedSensorPosition(0, 0, K_TimeoutMs);

    _talon4->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
    _talon4->SetSensorPhase(true);
    _talon4->ConfigNominalOutputForward(0, K_TimeoutMs);
    _talon4->ConfigNominalOutputReverse(0, K_TimeoutMs);
    _talon4->ConfigPeakOutputForward(1, K_TimeoutMs);
    _talon4->ConfigPeakOutputReverse(-1, K_TimeoutMs);
    _talon4->SetSelectedSensorPosition(0, 0, K_TimeoutMs);

}

void Robot::RobotPeriodic() {
  
  
    _talon6->SetSelectedSensorPosition(0, 0, K_TimeoutMs);
    _talon5->SetSelectedSensorPosition(0, 0, K_TimeoutMs);
    DesiredPos_Backward = 0;
    DesiredPos_Forward = 0;

    while(IsEnabled() && IsOperatorControl()){
      if(IsAuton){
          AutonDriveLiftWheel(_spark1);
      } else {
      //Back Lift Pos 
      Lift_Pos[E_RobotLiftBack] = _talon6->GetSelectedSensorPosition();
      Lift_Pos[E_RobotLiftForward] = _talon5->GetSelectedSensorPosition() * -1;
      SmartDashboard::PutNumber("Back Lift", Lift_Pos[E_RobotLiftBack]);
      SmartDashboard::PutNumber("Forward Lift", Lift_Pos[E_RobotLiftForward]);
      
      //y
      if(_joy1->GetRawButton(4)){
        DesiredPos_Backward -= 100;
        DesiredPos_Forward -= 100;
      }
      //x
      if(_joy1->GetRawButton(3)){
        DesiredPos_Forward += 100;
      }
      //b
      if(_joy1->GetRawButton(2)){
        DesiredPos_Backward += 100;
      }
      //a
      if(_joy1->GetRawButton(1)){
        DesiredPos_Backward += 100;
        DesiredPos_Forward += 100;
      }

      //DesiredPos Limit
      if (DesiredPos_Backward < -15500.0){
        DesiredPos_Backward = -15500.0;
      } 
      else if (DesiredPos_Backward > 0)
      {
        DesiredPos_Backward = 0;
      }

      if (DesiredPos_Forward < -15500.0){
        DesiredPos_Forward = -15500.0;
      } 
      else if (DesiredPos_Forward > 0)
      {
        DesiredPos_Forward = 0;
      }
      
      SmartDashboard::PutNumber("Desired Back", DesiredPos_Backward);
      SmartDashboard::PutNumber("Desired forward", DesiredPos_Forward);

      //Control Output
      double LiftOut_Backward = Control_PID(DesiredPos_Backward,
                            Lift_Pos[E_RobotLiftBack],
                            &ErrPrev_Lift[E_RobotLiftBack], 
                            &IntPrev_Lift[E_RobotLiftBack],
                            0.001, 0.0, 0.0, //P I D 
                            1, -0.75,    //P Upper and lower
                            1.0, -0.1,    //I Upper and lower
                            0,0,          //D Upper and lower
                            1, -0.75); //Out Upper and lower

      double LiftOut_Forward = Control_PID(DesiredPos_Forward,
                            Lift_Pos[E_RobotLiftForward],
                            &ErrPrev_Lift[E_RobotLiftForward], 
                            &IntPrev_Lift[E_RobotLiftForward],
                            0.001, 0.0, 0.0, //P I D 
                            1, -0.75,    //P Upper and lower
                            1.0, -0.1,    //I Upper and lower
                            0,0,          //D Upper and lower
                            1, -0.75); //Out Upper and lower
                      

      SmartDashboard::PutNumber("Lift Out Back", LiftOut_Backward);
      SmartDashboard::PutNumber("Lift Out Forward", LiftOut_Forward);

      //Set Motor Output
      _talon5->Set(ControlMode::PercentOutput, LiftOut_Forward * -1);
      _talon6->Set(ControlMode::PercentOutput, LiftOut_Backward);

      //Tank Drive

      double L_axis = _joy1->GetRawAxis(1) * -1;
      double R_axis = _joy1->GetRawAxis(5);

      if(L_axis > 0.5){
        L_axis = 0.5;
      }

      if(R_axis > 0.5){
        R_axis = 0.5;
      }
      
      //Dead Band
      // if(L_axis < 0.05 || L_axis > -0.05) {
      //   L_axis = 0;
      // }
      // if(R_axis < 0.05 || R_axis > -0.05) {
      //   R_axis = 0;
      // }

      if(Lift_Pos[E_RobotLiftBack] < -75 || Lift_Pos[E_RobotLiftForward] < -75){
        L_axis = L_axis * 0.25;
        R_axis = R_axis * 0.25;
      } 

      _talon1->Set(ControlMode::PercentOutput, L_axis);
      _talon2->Set(ControlMode::PercentOutput, L_axis);
      
      _talon3->Set(ControlMode::PercentOutput, R_axis);
      _talon4->Set(ControlMode::PercentOutput, R_axis);

      if(_joy1->GetRawButton(5)){
        _spark1->Set(0.5);
      } else {
        _spark1->Set(0);
      }

      if(_joy1->GetRawButton(8))
      {
        IsAuton = true;
      }

      Wait(C_ExeTime);
      }
    }
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {

  
  while (IsAutonomous() && IsEnabled()){
    // Lift_BackPos = _talon6->GetSelectedSensorPosition();
    // Lift_FrountPos = _talon5->GetSelectedSensorPosition() * -1;
    // SmartDashboard::PutNumber("Back Lift", Lift_BackPos);
    // SmartDashboard::PutNumber("Forward Lift", Lift_FrountPos);

    // double LiftOut_Backward = Control_PID(DesiredPos_Backward,
    //                         Lift_BackPos,
    //                         &ErrPrev_LiftBackward, 
    //                         &IntPrev_LifeBackward,
    //                         0.001, 0.0, 0.0, //P I D 
    //                         0.5, -0.5,    //P Upper and lower
    //                         1.0, -0.1,    //I Upper and lower
    //                         0,0,          //D Upper and lower
    //                         0.5, -0.5); //Out Upper and lower

    //   double LiftOut_Forward = Control_PID(DesiredPos_Forward,
    //                         Lift_FrountPos,
    //                         &ErrPrev_LiftForward, 
    //                         &IntPrev_LiftForward,
    //                         0.001, 0.0, 0.0, //P I D 
    //                         0.5, -0.5,    //P Upper and lower
    //                         1.0, -0.1,    //I Upper and lower
    //                         0,0,          //D Upper and lower
    //                         0.5, -0.5); //Out Upper and lower

    // //Start Lift
    // _talon5->Set(ControlMode::PercentOutput, LiftOut_Forward * -1);
    // _talon6->Set(ControlMode::PercentOutput, LiftOut_Backward);

    //Drive small Forward for time

    //Lift frount

    //Drive forward for encoder 

    //Lift Back

    Wait(0.01);
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
