
#include "Control_Pid.h"
#include "enums.h"

#include <iostream>
#include <string>

#include <ctre/Phoenix.h>
#include <frc/Ultrasonic.h>
#include <frc/Spark.h>

#include "const.h"

using namespace frc;

double Lift_ErrPrev[E_RobotLiftSz] = {0,0};
double Lift_IntPrev[E_RobotLiftSz] = {0,0};
double Lift_DesiredPos[E_RobotLiftSz] = {0,0};


double Drive_RawRPM[E_RobotSideSz] = {0,0};
double Drive_ErrPrev[E_RobotSideSz] = {0,0};
double Drive_IntPrev[E_RobotSideSz] = {0,0};

Ultrasonic *_UltraForward;
Ultrasonic *_UltraBack;

double AutonDesiredDriveRPM = 25;

bool AutonLiftToHight(TalonSRX *_talon6, TalonSRX *_talon5)
{
    double Lift_BackPos = _talon6->GetSelectedSensorPosition();
    double Lift_ForwardPos = _talon5->GetSelectedSensorPosition() * -1;

    Lift_DesiredPos[E_RobotLiftBack] -= 50;
    Lift_DesiredPos[E_RobotLiftForward] -= 50;

    double LiftOut_Backward = Control_PID(Lift_DesiredPos[E_RobotLiftBack],
                            Lift_BackPos,
                            &Lift_ErrPrev[E_RobotLiftBack], 
                            &Lift_IntPrev[E_RobotLiftBack],
                            0.001, 0.0, 0.0, //P I D 
                            0.5, -0.5,    //P Upper and lower
                            1.0, -0.1,    //I Upper and lower
                            0,0,          //D Upper and lower
                            0.5, -0.5); //Out Upper and lower

    double LiftOut_Forward = Control_PID(Lift_DesiredPos[E_RobotLiftForward],
                            Lift_ForwardPos,
                            &Lift_ErrPrev[E_RobotLiftForward], 
                            &Lift_IntPrev[E_RobotLiftForward],
                            0.001, 0.0, 0.0, //P I D 
                            0.5, -0.5,    //P Upper and lower
                            1.0, -0.1,    //I Upper and lower
                            0,0,          //D Upper and lower
                            0.5, -0.5); //Out Upper and lower
    
    if(Lift_BackPos > -15500){
        _talon6->Set(ControlMode::PercentOutput, LiftOut_Backward);
    } else {
        _talon6->Set(ControlMode::PercentOutput, 0);
    }

    if(Lift_ForwardPos > -15500){
        _talon5->Set(ControlMode::PercentOutput, LiftOut_Forward * -1);
    } else {
        _talon5->Set(ControlMode::PercentOutput, 0);
    }
    
    if(Lift_BackPos > -15500 && Lift_ForwardPos > -15500){
        return true;
    } else {
        return false;
    }
}

bool AutonDriveLiftWheel(Spark *_spark1)
{
    // _UltraForward = new Ultrasonic(7, 6);
    // _UltraForward->SetAutomaticMode(true);

    // double UltraDistance = _UltraForward->GetRangeInches();

    if(true){
       _spark1->Set(0.5);
       return false;
    } else {
        return true;
    }
}

bool AutonMainDrive(TalonSRX *_talon1, TalonSRX *_talon2, TalonSRX *_talon3, TalonSRX *_talon4)
{
    _UltraForward = new Ultrasonic(9, 8);
    _UltraForward->SetAutomaticMode(true);

    double UltraDistance = _UltraForward->GetRangeInches();

    Drive_RawRPM[E_RobotSideLeft]  = _talon2->GetSelectedSensorVelocity(0) / K_WheelPulseToRev;
    Drive_RawRPM[E_RobotSideRight]  = _talon4->GetSelectedSensorVelocity(0) / K_WheelPulseToRev;

    double DriveLeft = Control_PID(AutonDesiredDriveRPM,
                                    Drive_RawRPM[E_RobotSideLeft],
                                    &Drive_ErrPrev[E_RobotSideLeft], 
                                    &Drive_IntPrev[E_RobotSideLeft],
                                    0.001, 0.0, 0.0, //P I D 
                                    1, -0.75,    //P Upper and lower
                                    1.0, -0.1,    //I Upper and lower
                                    0,0,          //D Upper and lower
                                    1, -0.75); //Out Upper and lower

    double DriveRight = Control_PID(AutonDesiredDriveRPM,
                                    Drive_RawRPM[E_RobotSideRight],
                                    &Drive_ErrPrev[E_RobotSideRight], 
                                    &Drive_IntPrev[E_RobotSideRight],
                                    0.001, 0.0, 0.0, //P I D 
                                    1, -0.75,    //P Upper and lower
                                    1.0, -0.1,    //I Upper and lower
                                    0,0,          //D Upper and lower
                                    1, -0.75); //Out Upper and lower

    if(UltraDistance < 12){
        _talon1->Set(ControlMode::PercentOutput, 0);
        _talon2->Set(ControlMode::PercentOutput, 0);

        _talon3->Set(ControlMode::PercentOutput, 0);
        _talon4->Set(ControlMode::PercentOutput, 0);
        return true;
    } else {
        _talon1->Set(ControlMode::PercentOutput, DriveLeft);
        _talon2->Set(ControlMode::PercentOutput, DriveLeft);

        _talon3->Set(ControlMode::PercentOutput, DriveRight);
        _talon4->Set(ControlMode::PercentOutput, DriveRight);
        return false;
    }
    
}

bool AutonRaiseBackLift(TalonSRX *_talon6) 
{
    double Lift_BackPos = _talon6->GetSelectedSensorPosition();

    double Lift_Desired = 0;
    double LiftOut_Backward = Control_PID(Lift_Desired,
                            Lift_BackPos,
                            &Lift_ErrPrev[E_RobotLiftBack], 
                            &Lift_IntPrev[E_RobotLiftBack],
                            0.001, 0.0, 0.0, //P I D 
                            0.5, -0.5,    //P Upper and lower
                            1.0, -0.1,    //I Upper and lower
                            0,0,          //D Upper and lower
                            0.5, -0.5); //Out Upper and lower

    if(Lift_BackPos >= 0)
    {
        _talon6->Set(ControlMode::PercentOutput, 0);
        return true;
    } else {
        _talon6->Set(ControlMode::PercentOutput, LiftOut_Backward);
        return false;
    }
}

bool AutonRaiseForwardLift(TalonSRX *_talon5)
{
    double Lift_ForwardPos = _talon5->GetSelectedSensorPosition() * -1; 

    double Lift_Desired = 0;
    double LiftOut_Backward = Control_PID(Lift_Desired,
                            Lift_ForwardPos,
                            &Lift_ErrPrev[E_RobotLiftForward], 
                            &Lift_IntPrev[E_RobotLiftForward],
                            0.001, 0.0, 0.0, //P I D 
                            0.5, -0.5,    //P Upper and lower
                            1.0, -0.1,    //I Upper and lower
                            0,0,          //D Upper and lower
                            0.5, -0.5); //Out Upper and lower
    
    if(Lift_ForwardPos >= 0)
    {
        _talon5->Set(ControlMode::PercentOutput, 0);
        return true;
    } else {
        _talon5->Set(ControlMode::PercentOutput, LiftOut_Backward * -1);
        return false;
    }
}
