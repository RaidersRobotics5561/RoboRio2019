/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Control_Pid.h"
#include "Calibrations.hpp"
#include <frc/Driverstation.h>

using namespace frc;

double             V_RobotShimmyRightTime;
double             V_RobotShimmyLeftTime;
double             V_SwingScale;
T_RobotShimmyLeft  V_RobotShimmyLeft;
T_RobotShimmyRight V_RobotShimmyRight;

void Robot::RobotInit() {
  V_RobotShimmyRightTime  = 0.0;
  V_RobotShimmyLeftTime   = 0.0;
  V_SwingScale = 0.75;
  V_RobotShimmyLeft  = E_RobotShimmyLeft_RightBackwards;
  V_RobotShimmyRight = E_RobotShimmyRight_LeftBackwards;

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

    _UltraBack = new Ultrasonic(0, 1);
    _UltraBack->SetAutomaticMode(true);

    _UltraFront = new Ultrasonic(2, 3);
    _UltraFront->SetAutomaticMode(true);
}

void Robot::RobotPeriodic() {
    _talon6->SetSelectedSensorPosition(0, 0, K_TimeoutMs);
    _talon5->SetSelectedSensorPosition(0, 0, K_TimeoutMs);

    _talon2->SetSelectedSensorPosition(0, 0, K_TimeoutMs);
    _talon4->SetSelectedSensorPosition(0, 0, K_TimeoutMs);
    DesiredPos_Backward = 0;
    DesiredPos_Forward = 0;

    double Drive_Desired[E_RobotSideSz] = {0,0};
    double Drive_RPMRaw[E_RobotSideSz] = {0,0};
    double Drive_ErrPrev[E_RobotSideSz] = {0,0};
    double Drive_IntPrev[E_RobotSideSz] = {0,0};
    
    double Drive_SpeedGain = 0;
    double Drive_SpeedGain_Min = 0.4;

    double LiftOut_Forward = 0;
    double Drive_Left = 0;
    double Drive_Right = 0;
    double V_RobotUserCmndPct[E_RobotSideSz] = {0,0};
    double LiftOut_Backward = 0;

    int AutonStep = 0;
    bool Lifted = false;
    bool test = false;
    IsAuton = false;
    bool autonComp[9] = {false,false,false,false,false,false,false,false,false};
    while(IsEnabled() && IsOperatorControl()){
      if(test){
        double UltraDistance = _UltraFront->GetRangeInches();

        SmartDashboard::PutNumber("UltraDistance", UltraDistance);
        Wait(C_ExeTime);
      }else if(IsAuton){
        if(autonComp[0] == false){
          autonComp[0] = AutonLiftToHight(_talon6, _talon5);
        } else if(autonComp[1] == false) {
          autonComp[1] = AutonDriveLiftWheel(_spark1, _UltraFront);
          MaintainBackLift(_talon6);
          MaintainForwardLift(_talon5);
        } else if(autonComp[2] == false) {
          autonComp[2] = AutonRaiseForwardLift(_talon5);
          MaintainBackLift(_talon6);
        } else if(autonComp[3] == false) {
          autonComp[3] = AutonMainDrive(_talon1,_talon2,_talon3,_talon4,_UltraBack);
          MaintainBackLift(_talon6);
        } else if(autonComp[4] == false) {
          autonComp[4] = AutonRaiseBackLift(_talon6);
        }  else if(autonComp[4] == true){
          IsAuton = false;
        }
        SmartDashboard::PutBoolean("Step 0", autonComp[0]);
        SmartDashboard::PutBoolean("Step 1", autonComp[1]);
        SmartDashboard::PutBoolean("Step 2", autonComp[2]);
        SmartDashboard::PutBoolean("Step 3", autonComp[3]);
        SmartDashboard::PutBoolean("Step 4", autonComp[4]);
        Wait(C_ExeTime);
      } else {
      //Back Lift Pos
      Lift_Pos[E_RobotLiftBack] = _talon6->GetSelectedSensorPosition(); //FLIP ME TOO
      Lift_Pos[E_RobotLiftForward] = _talon5->GetSelectedSensorPosition() * -1;
      
      SmartDashboard::PutNumber("LiftPos Back:", Lift_Pos[E_RobotLiftBack]);
      SmartDashboard::PutNumber("LiftPos Forward:", Lift_Pos[E_RobotLiftForward]);
      
      Drive_RPMRaw[E_RobotSideLeft] = (_talon2->GetSelectedSensorVelocity(0) * 600) / K_WheelPulseToRev;
      Drive_RPMRaw[E_RobotSideRight] = ((_talon4->GetSelectedSensorVelocity(0) * 600) / K_WheelPulseToRev) * -1;

      SmartDashboard::PutNumber("Drive RPM Left:", Drive_RPMRaw[E_RobotSideLeft]);
      SmartDashboard::PutNumber("Drive RPM Right:", Drive_RPMRaw[E_RobotSideRight]);

      SmartDashboard::PutNumber("Ultra Front:", _UltraFront->GetRangeInches());
      SmartDashboard::PutNumber("Ultra Back:", _UltraBack->GetRangeInches());

      //y
      if(_joy2->GetRawButton(4)){
        DesiredPos_Backward -= 100;
        DesiredPos_Forward -= 100;
      }
      //x
      if(_joy2->GetRawButton(3)){
        DesiredPos_Forward += 100;
      }
      //b
      if(_joy2->GetRawButton(2)){
        DesiredPos_Backward += 100;
      }
      //a
      if(_joy2->GetRawButton(1)){
        DesiredPos_Backward += 100;
        DesiredPos_Forward += 100;
      }
      //up
      if(_joy2->GetPOV() == 0){
        DesiredPos_Forward -= 100;
      }
      //down
      if(_joy2->GetPOV() == 180){
        DesiredPos_Backward -= 100;
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

      SmartDashboard::PutNumber("Usr Cmd Left:", V_RobotUserCmndPct[E_RobotSideLeft]);
      SmartDashboard::PutNumber("Usr Cmd Right:", V_RobotUserCmndPct[E_RobotSideRight]);

      if(_joy1->GetRawAxis(3) < Drive_SpeedGain_Min)
      {
        Drive_SpeedGain = Drive_SpeedGain_Min;
      } else {
        Drive_SpeedGain = _joy1->GetRawAxis(3);
      }

      V_RobotUserCmndPct[E_RobotSideLeft] = -((_joy1->GetRawAxis(1) - (_joy1->GetRawAxis(4) * K_RotateGain))) * Drive_SpeedGain;
      V_RobotUserCmndPct[E_RobotSideRight] = -((_joy1->GetRawAxis(1) + (_joy1->GetRawAxis(4) * K_RotateGain))) * Drive_SpeedGain;

      if (_joy1->GetPOV() == 270)
        {
        /* Shimmy to the left: */
        if ((V_RobotShimmyLeftTime < K_RobotShimmyTime) &&
            (V_RobotShimmyLeft < E_RobotShimmyLeft_ShimmySz))
          {
          V_RobotShimmyLeftTime += C_ExeTime;
          }
        else
          {
          V_RobotShimmyLeft = T_RobotShimmyLeft(int(V_RobotShimmyLeft) + 1);
          V_RobotShimmyLeftTime = 0;
          if (V_RobotShimmyLeft >= E_RobotShimmyLeft_ShimmySz)
            {
            V_RobotShimmyLeft = E_RobotShimmyLeft_RightBackwards;
            }
          }

        if ((V_RobotShimmyLeft == E_RobotShimmyLeft_RightBackwards) ||
            (V_RobotShimmyLeft == E_RobotShimmyLeft_RightForward))
          {
          Drive_Desired[E_RobotSideLeft] = 0.0;
          if (V_RobotShimmyLeft == E_RobotShimmyLeft_RightBackwards)
            {
            Drive_Desired[E_RobotSideRight] = -K_RobotShimmySpeed;
            }
          else
            {
            Drive_Desired[E_RobotSideRight] = K_RobotShimmySpeed;
            }
          }
        else
          {
          Drive_Desired[E_RobotSideRight] = 0.0;
          if (V_RobotShimmyLeft == E_RobotShimmyLeft_LeftBackwards)
            {
            Drive_Desired[E_RobotSideLeft] = -K_RobotShimmySpeed;
            }
          else
            {
            Drive_Desired[E_RobotSideLeft] = K_RobotShimmySpeed;
            }
          }
        }
      else if (_joy1->GetPOV() == 90)
        {
        /* Shimmy to the right: */
        if ((V_RobotShimmyRightTime < K_RobotShimmyTime) &&
            (V_RobotShimmyRight < E_RobotShimmyRight_ShimmySz))
          {
          V_RobotShimmyRightTime += C_ExeTime;
          }
        else
          {
          V_RobotShimmyRight = T_RobotShimmyRight(int(V_RobotShimmyRight) + 1);
          V_RobotShimmyRightTime = 0;
          if (V_RobotShimmyRight >= E_RobotShimmyRight_ShimmySz)
            {
            V_RobotShimmyRight = E_RobotShimmyRight_LeftBackwards;
            }
          }

        if ((V_RobotShimmyRight == E_RobotShimmyRight_LeftBackwards) ||
            (V_RobotShimmyRight == E_RobotShimmyRight_LeftForward))
          {
          Drive_Desired[E_RobotSideRight] = 0.0;
          if (V_RobotShimmyRight == E_RobotShimmyRight_LeftBackwards)
            {
            Drive_Desired[E_RobotSideLeft] = -K_RobotShimmySpeed;
            }
          else
            {
            Drive_Desired[E_RobotSideLeft] = K_RobotShimmySpeed;
            }
          }
        else
          {
          Drive_Desired[E_RobotSideLeft] = 0.0;
          if (V_RobotShimmyRight == E_RobotShimmyRight_RightBackwards)
            {
            Drive_Desired[E_RobotSideRight] = -K_RobotShimmySpeed;
            }
          else
            {
            Drive_Desired[E_RobotSideRight] = K_RobotShimmySpeed;
            }
          }
        }
      else if(Lift_Pos[E_RobotLiftBack] < -75 || Lift_Pos[E_RobotLiftForward] < -75 || Lifted)
        {
        V_RobotShimmyRightTime  = 0.0;
        V_RobotShimmyLeftTime   = 0.0;
        V_RobotShimmyLeft = E_RobotShimmyLeft_RightBackwards;
        V_RobotShimmyRight = E_RobotShimmyRight_LeftBackwards;
        Lifted = true;
        Drive_Desired[E_RobotSideLeft] = DesiredSpeed(V_RobotUserCmndPct[E_RobotSideLeft], Drive_RPMRaw[E_RobotSideLeft]);
        Drive_Desired[E_RobotSideRight] = DesiredSpeed(V_RobotUserCmndPct[E_RobotSideRight], Drive_RPMRaw[E_RobotSideRight]);
        }
      else
        {
        V_RobotShimmyRightTime  = 0.0;
        V_RobotShimmyLeftTime   = 0.0;
        V_RobotShimmyLeft = E_RobotShimmyLeft_RightBackwards;
        V_RobotShimmyRight = E_RobotShimmyRight_LeftBackwards;
        Drive_Desired[E_RobotSideLeft] = DesiredSpeed(V_RobotUserCmndPct[E_RobotSideLeft], Drive_RPMRaw[E_RobotSideLeft]);
        Drive_Desired[E_RobotSideRight] = DesiredSpeed(V_RobotUserCmndPct[E_RobotSideRight], Drive_RPMRaw[E_RobotSideRight]);
        }


      //Control Output
      LiftOut_Backward = Control_PID(DesiredPos_Backward,
                                     Lift_Pos[E_RobotLiftBack],
                                     &ErrPrev_Lift[E_RobotLiftBack],
                                     &IntPrev_Lift[E_RobotLiftBack],
                                     0.001, 0.0, 0.0, //P I D
                                     1, -0.75,    //P Upper and lower
                                     1.0, -0.1,    //I Upper and lower
                                     0,0,          //D Upper and lower
                                     1, -0.75); //Out Upper and lower

      LiftOut_Forward = Control_PID(DesiredPos_Forward,
                            Lift_Pos[E_RobotLiftForward],
                            &ErrPrev_Lift[E_RobotLiftForward],
                            &IntPrev_Lift[E_RobotLiftForward],
                            0.001, 0.0, 0.0, //P I D
                            1, -0.75,    //P Upper and lower
                            1.0, -0.1,    //I Upper and lower
                            0,0,          //D Upper and lower
                            1, -0.75); //Out Upper and lower

      Drive_Left = Control_PID(Drive_Desired[E_RobotSideLeft],
                            Drive_RPMRaw[E_RobotSideLeft],
                            &Drive_ErrPrev[E_RobotSideLeft],
                            &Drive_IntPrev[E_RobotSideLeft],
                            0.002, 0.0002, 0.00002, //P I D
                            0.8, -0.8,    //P Upper and lower
                            1.0, -1,    //I Upper and lower
                            1,-1,          //D Upper and lower
                            1, -1); //Out Upper and lower

      Drive_Right = -1 * Control_PID(Drive_Desired[E_RobotSideRight],
                            Drive_RPMRaw[E_RobotSideRight],
                            &Drive_ErrPrev[E_RobotSideRight],
                            &Drive_ErrPrev[E_RobotSideRight],
                            0.002, 0.0002, 0.00002, //P I D
                            0.8, -0.8,    //P Upper and lower
                            1.0, -1,    //I Upper and lower
                            1,-1,          //D Upper and lower
                            1, -1); //Out Upper and lower

      SmartDashboard::PutNumber("Drive left:", Drive_Left);
      SmartDashboard::PutNumber("Drive right:", Drive_Right);
      SmartDashboard::PutNumber("Usr Cmd Left:", V_RobotUserCmndPct[E_RobotSideLeft]);
      SmartDashboard::PutNumber("Usr Cmd Right:", V_RobotUserCmndPct[E_RobotSideRight]);
      SmartDashboard::PutNumber("Drive Desired Left:", Drive_Desired[E_RobotSideLeft]);
      SmartDashboard::PutNumber("Drive Desored Right:", Drive_Desired[E_RobotSideRight]);

      SmartDashboard::PutNumber("POV:", _joy1->GetPOV());

      //Set Motor Output
      _talon5->Set(ControlMode::PercentOutput, LiftOut_Forward * -1);
      _talon6->Set(ControlMode::PercentOutput, LiftOut_Backward);

      //Tank Drive


      _talon1->Set(ControlMode::PercentOutput, Drive_Left);
      _talon2->Set(ControlMode::PercentOutput, Drive_Left);

      _talon3->Set(ControlMode::PercentOutput, Drive_Right);
      _talon4->Set(ControlMode::PercentOutput, Drive_Right);


      // if(_joy2->GetPOV() == 90){
      //   _spark1->Set(1);
      // } else if(_joy2->GetPOV() == 270) {
      //   _spark1->Set(-1);
      // }
      // else {
      //   _spark1->Set(0);
      // }

       if(_joy2->GetRawAxis(0)  > 0.05){
        _spark1->Set(_joy2->GetRawAxis(0));
      } else if(_joy2->GetRawAxis(0)  < -0.05){
        _spark1->Set(_joy2->GetRawAxis(0) * -1);
      } else {
        _spark1->Set(0);
      }

      if(_joy2->GetRawAxis(3) * V_SwingScale > 0.05){
        _spark2->Set(_joy2->GetRawAxis(3) * V_SwingScale);
      }

      if(_joy2->GetRawAxis(2) * V_SwingScale > 0.05){
        _spark2->Set((_joy2->GetRawAxis(2) * V_SwingScale) * -1);
      }

      // if(_joy2->GetPOV() == 0){
      //   _spark2->Set(0.75);
      // } else if(_joy2->GetPOV() == 180){
      //   _spark2->Set(-0.5);
      // } else {
      //   _spark2->Set(0);
      // }

      if(_joy2->GetRawButton(5)){
        _spark3->Set(0.5);
      } else if(_joy2->GetRawButton(6)){
        _spark3->Set(-0.5);
      } else {
        _spark3->Set(0);
      }

      if(_joy1->GetRawButton(8))
      {
        IsAuton = true;
      }

      if(DriverStation::GetInstance().IsBrownedOut())
      {
        _talon5->Set(ControlMode::PercentOutput, 0);
        _talon6->Set(ControlMode::PercentOutput, 0);
      }

      Wait(C_ExeTime);
      }
    }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
