/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Control_Pid.h"
#include "Calibrations.hpp"


using namespace frc;

double             V_SwingScale;
T_RobotShimmyLeft  V_RobotShimmyLeft;
T_RobotShimmyRight V_RobotShimmyRight;

T_RoboState V_RobotState;

void UpdateLED_Output(T_RoboState    L_RobotState,
                      bool           L_DriverOverride,
                      DigitalOutput *L_LED_State0,
                      DigitalOutput *L_LED_State1,
                      DigitalOutput *L_LED_State2,
                      DigitalOutput *L_LED_State3);

void Robot::RobotInit() {
  V_RobotState = E_Init;
  V_SwingScale = 0.75;
  V_RobotShimmyLeft  = E_RobotShimmyLeft_ShimmySz;
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

    UpdateLED_Output(V_RobotState,
                     false,
                     V_LED_State0,
                     V_LED_State1,
                     V_LED_State2,
                     V_LED_State3);
}

void Robot::RobotPeriodic() {
    _talon6->SetSelectedSensorPosition(0, 0, K_TimeoutMs);
    _talon5->SetSelectedSensorPosition(0, 0, K_TimeoutMs);

    _talon2->SetSelectedSensorPosition(0, 0, K_TimeoutMs);
    _talon4->SetSelectedSensorPosition(0, 0, K_TimeoutMs);
    DesiredPos_Backward = 0;
    DesiredPos_Forward = 0;

    V_RobotState = E_Teleop;

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

    double V_RobotShimmyDistance = 0;
    bool   V_RobotShimmyLeftInProcess  = false;
    bool   V_RobotShimmyRightInProcess = false;
    bool   V_RobotShimmyStop           = false;
    bool   L_Test = false;
    double L_OpenLoopTimer = 0;

    int AutonStep = 0;
    bool test = true;
    IsAuton = false;
    IsAutonDwn = false;
    bool autonStagesClimb[9] = {false,false,false,false,false,false,false,false,false};
    bool autonStagesClimbDown[9] = {false,false,false,false,false,false,false,false,false};
    while(IsEnabled() && IsOperatorControl()){
    
    /* Let's update the LED ligts here so that it will always run, regardless of in auto mode or in driver mode. */
    UpdateLED_Output(V_RobotState,
                     _joy1->GetRawButton(2),
                     V_LED_State0,
                     V_LED_State1,
                     V_LED_State2,
                     V_LED_State3);

      if(IsAutonDwn)
        {
        if (autonStagesClimbDown[0] == false)
          {
          // With the front overhanging, drop the back lift
          autonStagesClimbDown[0] = AutonDropBackLift(_talon6, K_LiftHeightStage2, K_LiftHeightStage2);
          }
        else if(autonStagesClimbDown[1] == false)
          {
          // Hold the back lift and then start to drive off the platform until the front ultrasonic detects the edge
          MaintainBackLift(_talon6, K_LiftHeightStage2);
          autonStagesClimbDown[1] = AutonMainDriveRev(_talon1,_talon2,_talon3,_talon4,_UltraFront, -50);
          }
        else if(autonStagesClimbDown[2] == false)
          {
          // Now lets stop the drive wheels and then drop the front lift while still holding the back lift
          MaintainBackLift(_talon6, K_LiftHeightStage2);
          L_Test = AutonMainDriveRev(_talon1,_talon2,_talon3,_talon4,_UltraFront, 0);
          autonStagesClimbDown[2] = AutonRaiseForwardDrop(_talon5, K_LiftHeightStage2, K_LiftHeightStage2);
          }
        else if(autonStagesClimbDown[3] == false)
          {
          // Great, we've got both lifts extended, lets drive backwards a bit (open loop!!)
          MaintainBackLift(_talon6, K_LiftHeightStage2);
          MaintainForwardLift(_talon5, K_LiftHeightStage2);
          L_OpenLoopTimer += C_ExeTime;
          autonStagesClimbDown[3] = AutonDriveLiftWheelOpenLoop(_spark1, L_OpenLoopTimer);
          }
        else if(autonStagesClimbDown[4] == false)
          {
          // We've driven backwards a bit, lets pull the lifts back and drop the robot
          autonStagesClimbDown[4] = AutonDropToHight(_talon6, _talon5, 0);
          }
        else if(autonStagesClimbDown[4] == true)
          {
          // All done!  We are back on the ground and can resume normal driving!
          IsAutonDwn = false;
          }
        Wait(C_ExeTime);
        }
      else if(IsAuton){
        if(autonStagesClimb[0] == false){
          autonStagesClimb[0] = AutonLiftToHight(_talon6, _talon5, K_LiftHeightStage3);
          V_RobotState = E_AutonEndGame1;
        } else if(autonStagesClimb[1] == false) {
          autonStagesClimb[1] = AutonDriveLiftWheel(_spark1, _UltraFront);
          MaintainBackLift(_talon6, K_LiftHeightStage3);
          MaintainForwardLift(_talon5, K_LiftHeightStage3);
          V_RobotState = E_AutonEndGame2;
        } else if(autonStagesClimb[2] == false) {
          autonStagesClimb[2] = AutonRaiseForwardLift(_talon5, 0.0);
          MaintainBackLift(_talon6, K_LiftHeightStage3);
          _talon1->Set(ControlMode::PercentOutput, 0.1);
          _talon2->Set(ControlMode::PercentOutput, 0.1);
          _talon3->Set(ControlMode::PercentOutput, -0.1);
          _talon4->Set(ControlMode::PercentOutput, -0.1);
          V_RobotState = E_AutonEndGame3;
        } else if(autonStagesClimb[3] == false) {
          autonStagesClimb[3] = AutonMainDrive(_talon1,_talon2,_talon3,_talon4,_UltraBack);
          MaintainBackLift(_talon6, K_LiftHeightStage3);
          V_RobotState = E_AutonEndGame4;
        } else if(autonStagesClimb[4] == false) {
          autonStagesClimb[4] = AutonRaiseBackLift(_talon6, 0.0, -150);
          V_RobotState = E_AutonEndGame5;
        }  else if(autonStagesClimb[4] == true) {
          IsAuton = false;
          V_RobotState = E_Teleop;
        }
        SmartDashboard::PutBoolean("Step 0", autonStagesClimb[0]);
        SmartDashboard::PutBoolean("Step 1", autonStagesClimb[1]);
        SmartDashboard::PutBoolean("Step 2", autonStagesClimb[2]);
        SmartDashboard::PutBoolean("Step 3", autonStagesClimb[3]);
        SmartDashboard::PutBoolean("Step 4", autonStagesClimb[4]);
        Wait(C_ExeTime);
      } else {
      V_RobotState = E_Teleop;

      //Back Lift Pos
      Lift_Pos[E_RobotLiftBack] = _talon6->GetSelectedSensorPosition() * K_RobotType; //FLIP ME TOO
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
      if (DesiredPos_Backward < K_LiftHeightStage3){
        DesiredPos_Backward = K_LiftHeightStage3;
      }
      else if (DesiredPos_Backward > 0)
      {
        DesiredPos_Backward = 0;
      }

      if (DesiredPos_Forward < K_LiftHeightStage3){
        DesiredPos_Forward = K_LiftHeightStage3;
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

      if ((_joy1->GetPOV() == 270) ||
          (V_RobotShimmyLeftInProcess == true))
        {
        /* Shimmy to the left: */
        V_RobotShimmyLeftInProcess = true;
        if (V_RobotShimmyLeft >= E_RobotShimmyLeft_ShimmySz)
          {
          V_RobotShimmyLeft = E_RobotShimmyLeft_RightBackwards;
          }

        if (((V_RobotShimmyLeft == E_RobotShimmyLeft_RightBackwards) ||
             (V_RobotShimmyLeft == E_RobotShimmyLeft_RightForward)) &&
            (V_RobotShimmyStop == false))
          {
          Drive_Desired[E_RobotSideLeft] = 0.0;
          if (V_RobotShimmyLeft == E_RobotShimmyLeft_RightBackwards)
            {
            Drive_Desired[E_RobotSideRight] = -K_RobotShimmySpeed;
            V_RobotShimmyDistance = fabs(double(_talon4->GetSelectedSensorPosition()));
            }
          else
            {
            Drive_Desired[E_RobotSideRight] = K_RobotShimmySpeed;
            V_RobotShimmyDistance = fabs(double(_talon4->GetSelectedSensorPosition()));
            }
          }
        else
          {
          Drive_Desired[E_RobotSideRight] = 0.0;
          if (V_RobotShimmyLeft == E_RobotShimmyLeft_LeftBackwards)
            {
            Drive_Desired[E_RobotSideLeft] = -K_RobotShimmySpeed;
            V_RobotShimmyDistance = fabs(double(_talon2->GetSelectedSensorPosition()));
            }
          else
            {
            Drive_Desired[E_RobotSideLeft] = K_RobotShimmySpeed;
            V_RobotShimmyDistance = fabs(double(_talon2->GetSelectedSensorPosition()));
            }
          }

        if ((V_RobotShimmyDistance >= K_RobotShimmyDistance) ||
            (V_RobotShimmyStop == true))
          {
          V_RobotShimmyStop = true;
          Drive_Desired[E_RobotSideRight] = 0.0;
          Drive_Desired[E_RobotSideLeft]  = 0.0;

          if ((fabs(Drive_RPMRaw[E_RobotSideRight]) < 40.0) &&
              (fabs(Drive_RPMRaw[E_RobotSideLeft]) < 40.0))
            {
            V_RobotShimmyStop = false;
            V_RobotShimmyLeft = T_RobotShimmyLeft(int(V_RobotShimmyLeft) + 1);
            _talon2->SetSelectedSensorPosition(0, 0, K_TimeoutMs);
            _talon4->SetSelectedSensorPosition(0, 0, K_TimeoutMs);
            V_RobotShimmyDistance = 0;
            }

          if (V_RobotShimmyLeft >= E_RobotShimmyLeft_ShimmySz)
            {
            V_RobotShimmyLeftInProcess = false;
            V_RobotShimmyLeft = E_RobotShimmyLeft_RightBackwards;
            }
          }
        }
      else if ((_joy1->GetPOV() == 90) ||
               (V_RobotShimmyRightInProcess == true))
        {
        /* Shimmy to the right: */
        V_RobotShimmyRightInProcess = true;
        if (V_RobotShimmyRight >= E_RobotShimmyRight_ShimmySz)
          {
          V_RobotShimmyRight = E_RobotShimmyRight_LeftBackwards;
          }

        if ((V_RobotShimmyRight == E_RobotShimmyRight_LeftBackwards) ||
            (V_RobotShimmyRight == E_RobotShimmyRight_LeftForward))
          {
          Drive_Desired[E_RobotSideRight] = 0.0;
          if (V_RobotShimmyRight == E_RobotShimmyRight_LeftBackwards)
            {
            Drive_Desired[E_RobotSideLeft] = -K_RobotShimmySpeed;
            V_RobotShimmyDistance = fabs(_talon2->GetSelectedSensorPosition());
            }
          else
            {
            Drive_Desired[E_RobotSideLeft] = K_RobotShimmySpeed;
            V_RobotShimmyDistance = fabs(_talon2->GetSelectedSensorPosition());
            }
          }
        else
          {
          Drive_Desired[E_RobotSideLeft] = 0.0;
          if (V_RobotShimmyRight == E_RobotShimmyRight_RightBackwards)
            {
            Drive_Desired[E_RobotSideRight] = -K_RobotShimmySpeed;
            V_RobotShimmyDistance = fabs(_talon4->GetSelectedSensorPosition());
            }
          else
            {
            Drive_Desired[E_RobotSideRight] = K_RobotShimmySpeed;
            V_RobotShimmyDistance = fabs(_talon4->GetSelectedSensorPosition());
            }
          }

        if (V_RobotShimmyDistance >= K_RobotShimmyDistance)
          {
          V_RobotShimmyRight = T_RobotShimmyRight(int(V_RobotShimmyRight) + 1);
          V_RobotShimmyDistance = 0;
          Drive_Desired[E_RobotSideRight] = 0.0;
          Drive_Desired[E_RobotSideLeft]  = 0.0;
          _talon2->SetSelectedSensorPosition(0, 0, K_TimeoutMs);
          _talon4->SetSelectedSensorPosition(0, 0, K_TimeoutMs);
          if (V_RobotShimmyRight >= E_RobotShimmyRight_ShimmySz)
            {
            V_RobotShimmyRightInProcess = false;
            V_RobotShimmyRight = E_RobotShimmyRight_LeftBackwards;
            }
          }
        }
      else
        {
        Drive_Desired[E_RobotSideLeft] = DesiredSpeed(V_RobotUserCmndPct[E_RobotSideLeft], Drive_RPMRaw[E_RobotSideLeft]);
        Drive_Desired[E_RobotSideRight] = DesiredSpeed(V_RobotUserCmndPct[E_RobotSideRight], Drive_RPMRaw[E_RobotSideRight]);
        V_RobotShimmyLeft = E_RobotShimmyLeft_ShimmySz;
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


      SmartDashboard::PutNumber("Shimmy Distance:", V_RobotShimmyDistance);
      SmartDashboard::PutNumber("Drive left:", Drive_Left);
      SmartDashboard::PutNumber("Drive right:", Drive_Right);
      SmartDashboard::PutNumber("Usr Cmd Left:", V_RobotUserCmndPct[E_RobotSideLeft]);
      SmartDashboard::PutNumber("Usr Cmd Right:", V_RobotUserCmndPct[E_RobotSideRight]);
      SmartDashboard::PutNumber("Drive Desired Left:", Drive_Desired[E_RobotSideLeft]);
      SmartDashboard::PutNumber("Drive Desored Right:", Drive_Desired[E_RobotSideRight]);

      SmartDashboard::PutNumber("POV:", _joy1->GetPOV());

      //Set Lift Motor Output
      _talon5->Set(ControlMode::PercentOutput, LiftOut_Forward * -1);
      _talon6->Set(ControlMode::PercentOutput, LiftOut_Backward * K_RobotType);

      //Tank Drive
      _talon1->Set(ControlMode::PercentOutput, Drive_Left);
      _talon2->Set(ControlMode::PercentOutput, Drive_Left);

      _talon3->Set(ControlMode::PercentOutput, Drive_Right);
      _talon4->Set(ControlMode::PercentOutput, Drive_Right);


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

      if(_joy2->GetRawButton(5)){
        _spark3->Set(0.5);
      } else if(_joy2->GetRawButton(6)){
        _spark3->Set(-0.5);
      } else {
        _spark3->Set(0);
      }

/* Let's determine if we want to run the auto up feature.  We only want to allow the auto up feature 
   if we've reached the end game warning (don't want to accidently trigger this mode). */
      if((_joy1->GetRawButton(8) == true) &&
         (DriverStation::GetInstance().GetMatchTime() <= K_EndMatchWarningTime))
      {
        IsAuton = true;
      }

/* Let's determine if we want to run the auto down feature.  We only want to allow the auto down 
   feature if in "sandstorm" (don't want to accidently trigger this mode). */
      if((_joy1->GetRawButton(1) == true) &&
         (DriverStation::GetInstance().GetMatchTime() >= K_SandStormTime))
      {
        IsAutonDwn = true;
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
