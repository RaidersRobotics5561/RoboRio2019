#include "Control_Pid.h"
// #include "enums.h"
#include "Calibrations.hpp"

#include <iostream>
#include <string>

#include <ctre/Phoenix.h>
#include <frc/Ultrasonic.h>
#include <frc/Spark.h>

// #include "const.h"


using namespace frc;

double Lift_ErrPrev[E_RobotLiftSz] = {0,0};
double Lift_IntPrev[E_RobotLiftSz] = {0,0};
double Lift_DesiredPos[E_RobotLiftSz] = {0,0};


double AutonDrive_RawRPM[E_RobotSideSz] = {0,0};
double AutonDrive_ErrPrev[E_RobotSideSz] = {0,0};
double AutonDrive_IntPrev[E_RobotSideSz] = {0,0};

Ultrasonic *_UltraForward;
Ultrasonic *_UltraBack;

double AutonDesiredDriveRPM = 50;
double LiftThresh = -400;
double AutonPID_P[E_RobotSideSz];


bool AutonLiftToHight(TalonSRX *_talon6, TalonSRX *_talon5, double liftTarget)
{
    double Lift_BackPos = _talon6->GetSelectedSensorPosition() * K_RobotType;
    double Lift_ForwardPos = _talon5->GetSelectedSensorPosition() * -1;

    SmartDashboard::PutNumber("Lift pos back", Lift_BackPos);
    SmartDashboard::PutNumber("Lift pos forward", Lift_ForwardPos);

   
    AutonPID_P[E_RobotLiftForward] = 1 - (Lift_ForwardPos - Lift_BackPos)/LiftThresh;
    AutonPID_P[E_RobotLiftBack] = 1 - (Lift_BackPos - Lift_ForwardPos)/LiftThresh;
    
    if(AutonPID_P[E_RobotLiftForward] <= 0){
        AutonPID_P[E_RobotLiftForward] = 0; 
    }

    if(AutonPID_P[E_RobotLiftBack] <= 0){
        AutonPID_P[E_RobotLiftBack] = 0; 
    }

    Lift_DesiredPos[E_RobotLiftBack] = liftTarget;
    Lift_DesiredPos[E_RobotLiftForward] = liftTarget;

    SmartDashboard::PutNumber("Lift_DesiredPos Forward", Lift_DesiredPos[E_RobotLiftForward]);
    SmartDashboard::PutNumber("Lift_DesiredPos Back", Lift_DesiredPos[E_RobotLiftBack]);

    double LiftOut_Backward = Control_PID(Lift_DesiredPos[E_RobotLiftBack],
                            Lift_BackPos,
                            &Lift_ErrPrev[E_RobotLiftBack], 
                            &Lift_IntPrev[E_RobotLiftBack],
                            0.0002* AutonPID_P[E_RobotLiftBack], 0.0005, 0.0, //P I D 
                            0.75, -0.75,    //P Upper and lower
                            1.0, -0.1,    //I Upper and lower
                            0,0,          //D Upper and lower
                            1, -1); //Out Upper and lower

    double LiftOut_Forward = Control_PID(Lift_DesiredPos[E_RobotLiftForward],
                            Lift_ForwardPos,
                            &Lift_ErrPrev[E_RobotLiftForward], 
                            &Lift_IntPrev[E_RobotLiftForward],
                            0.0004 * AutonPID_P[E_RobotLiftForward], 0.0005, 0.0, //P I D 
                            0.75, -0.75,    //P Upper and lower
                            1.0, -0.1,    //I Upper and lower
                            0,0,          //D Upper and lower
                            1, -1); //Out Upper and lower

    

    SmartDashboard::PutNumber("Lift out Back", LiftOut_Backward);
    SmartDashboard::PutNumber("Lift out forward", LiftOut_Forward);

    SmartDashboard::PutNumber("P LEFT", 0.0004 * AutonPID_P[E_RobotLiftForward]);
    SmartDashboard::PutNumber("P RIGHT", 0.0002 * AutonPID_P[E_RobotLiftBack]);
    
    if(Lift_BackPos > liftTarget){
        _talon6->Set(ControlMode::PercentOutput, LiftOut_Backward * K_RobotType);
    } else {
        _talon6->Set(ControlMode::PercentOutput, 0);
    }

    if(Lift_ForwardPos > liftTarget){
        _talon5->Set(ControlMode::PercentOutput, LiftOut_Forward * -1);
    } else {
        _talon5->Set(ControlMode::PercentOutput, 0);
    }
    
    if(Lift_BackPos < liftTarget + 750 && Lift_ForwardPos < liftTarget + 750){
        SmartDashboard::PutBoolean("Done", true);
        return true;
    } else {
        return false;
    }
}

bool AutonDropToHight(TalonSRX *_talon6, TalonSRX *_talon5, double liftTarget)
{
    double Lift_BackPos = _talon6->GetSelectedSensorPosition() * K_RobotType;
    double Lift_ForwardPos = _talon5->GetSelectedSensorPosition() * -1;

    SmartDashboard::PutNumber("Lift pos back", Lift_BackPos);
    SmartDashboard::PutNumber("Lift pos forward", Lift_ForwardPos);

 
    AutonPID_P[E_RobotLiftForward] = 1 - (Lift_ForwardPos - Lift_BackPos)/LiftThresh;
    AutonPID_P[E_RobotLiftBack] = 1 - (Lift_BackPos - Lift_ForwardPos)/LiftThresh;
    
    if(AutonPID_P[E_RobotLiftForward] <= 0){
        AutonPID_P[E_RobotLiftForward] = 0; 
    }

    if(AutonPID_P[E_RobotLiftBack] <= 0){
        AutonPID_P[E_RobotLiftBack] = 0; 
    }

    Lift_DesiredPos[E_RobotLiftBack] = liftTarget;
    Lift_DesiredPos[E_RobotLiftForward] = liftTarget;

    SmartDashboard::PutNumber("Lift_DesiredPos Forward", Lift_DesiredPos[E_RobotLiftForward]);
    SmartDashboard::PutNumber("Lift_DesiredPos Back", Lift_DesiredPos[E_RobotLiftBack]);

    double LiftOut_Backward = Control_PID(Lift_DesiredPos[E_RobotLiftBack],
                            Lift_BackPos,
                            &Lift_ErrPrev[E_RobotLiftBack], 
                            &Lift_IntPrev[E_RobotLiftBack],
                            0.0002* AutonPID_P[E_RobotLiftBack], 0.0005, 0.0, //P I D 
                            0.75, -0.75,    //P Upper and lower
                            1.0, -0.1,    //I Upper and lower
                            0,0,          //D Upper and lower
                            1, -1); //Out Upper and lower

    double LiftOut_Forward = Control_PID(Lift_DesiredPos[E_RobotLiftForward],
                            Lift_ForwardPos,
                            &Lift_ErrPrev[E_RobotLiftForward], 
                            &Lift_IntPrev[E_RobotLiftForward],
                            0.0004 * AutonPID_P[E_RobotLiftForward], 0.0005, 0.0, //P I D 
                            0.75, -0.75,    //P Upper and lower
                            1.0, -0.1,    //I Upper and lower
                            0,0,          //D Upper and lower
                            1, -1); //Out Upper and lower

    

    SmartDashboard::PutNumber("Lift out Back", LiftOut_Backward);
    SmartDashboard::PutNumber("Lift out forward", LiftOut_Forward);

    SmartDashboard::PutNumber("P LEFT", 0.0004 * AutonPID_P[E_RobotLiftForward]);
    SmartDashboard::PutNumber("P RIGHT", 0.0002 * AutonPID_P[E_RobotLiftBack]);
    
    if(Lift_BackPos <= liftTarget){
        _talon6->Set(ControlMode::PercentOutput, LiftOut_Backward * K_RobotType);
    } else {
        _talon6->Set(ControlMode::PercentOutput, 0);
    }

    if(Lift_ForwardPos <= liftTarget){
        _talon5->Set(ControlMode::PercentOutput, LiftOut_Forward * -1);
    } else {
        _talon5->Set(ControlMode::PercentOutput, 0);
    }
    
    if(Lift_BackPos >= liftTarget - 200 && Lift_ForwardPos >= liftTarget + 200){
        SmartDashboard::PutBoolean("Done", true);
        return true;
    } else {
        return false;
    }
}

bool AutonDriveLiftWheel(Spark *_spark1, Ultrasonic *_Ultrasonic)
{

    double UltraDistance = _Ultrasonic->GetRangeInches();

    SmartDashboard::PutNumber("UltraDistance", UltraDistance);

    if(UltraDistance >= 5){
       _spark1->Set(1);
       return false;
    } else {
        _spark1->Set(0.0);
        return true;
    }
}

bool AutonDriveLiftWheelOpenLoop(Spark *_spark1, double L_Time)
{
    if(L_Time < 0.75){
       _spark1->Set(-1.0);
       return false;
    } else {
        _spark1->Set(0.0);
        return true;
    }
}


bool AutonMainDrive(TalonSRX *_talon1, TalonSRX *_talon2, TalonSRX *_talon3, TalonSRX *_talon4, Ultrasonic *_Ultrasonic)
{
    double UltraDistance = _Ultrasonic->GetRangeInches();

    SmartDashboard::PutNumber("Ultra Main Drive", UltraDistance);
    
    AutonDrive_RawRPM[E_RobotSideLeft]  = (_talon2->GetSelectedSensorVelocity(0) * 600) / K_WheelPulseToRev;
    AutonDrive_RawRPM[E_RobotSideRight]  = ((_talon4->GetSelectedSensorVelocity(0) * 600) / K_WheelPulseToRev) * -1;

    double DriveLeft = Control_PID(AutonDesiredDriveRPM,
                                    AutonDrive_RawRPM[E_RobotSideLeft],
                                    &AutonDrive_ErrPrev[E_RobotSideLeft], 
                                    &AutonDrive_IntPrev[E_RobotSideLeft],
                                    0.001, 0.0003, 0.0, //P I D 
                                    1, -0.75,    //P Upper and lower
                                    1.0, -0.1,    //I Upper and lower
                                    0,0,          //D Upper and lower
                                    1, -0.75); //Out Upper and lower

    double DriveRight = -Control_PID(AutonDesiredDriveRPM,
                                    AutonDrive_RawRPM[E_RobotSideRight],
                                    &AutonDrive_ErrPrev[E_RobotSideRight], 
                                    &AutonDrive_IntPrev[E_RobotSideRight],
                                    0.001, 0.0003, 0.0, //P I D 
                                    1, -0.75,    //P Upper and lower
                                    1.0, -0.1,    //I Upper and lower
                                    0,0,          //D Upper and lower
                                    1, -0.75); //Out Upper and lower

    if(UltraDistance <= 5){
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

bool AutonMainDriveRev(TalonSRX *_talon1, TalonSRX *_talon2, TalonSRX *_talon3, TalonSRX *_talon4, Ultrasonic *_Ultrasonic, double L_Spd)
{
    double UltraDistance = _Ultrasonic->GetRangeInches();

    SmartDashboard::PutNumber("Ultra Main Drive Rev", UltraDistance);
    
    AutonDrive_RawRPM[E_RobotSideLeft]  = (_talon2->GetSelectedSensorVelocity(0) * 600) / K_WheelPulseToRev;
    AutonDrive_RawRPM[E_RobotSideRight]  = ((_talon4->GetSelectedSensorVelocity(0) * 600) / K_WheelPulseToRev) * -1;

    double DriveLeft = Control_PID(L_Spd,
                                    AutonDrive_RawRPM[E_RobotSideLeft],
                                    &AutonDrive_ErrPrev[E_RobotSideLeft], 
                                    &AutonDrive_IntPrev[E_RobotSideLeft],
                                    0.001, 0.0003, 0.0, //P I D 
                                    1, -1,    //P Upper and lower
                                    1.0, -1.0,    //I Upper and lower
                                    0,0,          //D Upper and lower
                                    1, -1); //Out Upper and lower

    double DriveRight = -Control_PID(L_Spd,
                                    AutonDrive_RawRPM[E_RobotSideRight],
                                    &AutonDrive_ErrPrev[E_RobotSideRight], 
                                    &AutonDrive_IntPrev[E_RobotSideRight],
                                    0.001, 0.0003, 0.0, //P I D 
                                    1, -1,    //P Upper and lower
                                    1.0, -1.0,    //I Upper and lower
                                    0,0,          //D Upper and lower
                                    1, -1); //Out Upper and lower

    if(UltraDistance >= 5){
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

bool AutonRaiseBackLift(TalonSRX *_talon6,
                        double   L_Pos,
                        double   L_End) 
{
    double Lift_BackPos = _talon6->GetSelectedSensorPosition() * K_RobotType;

    SmartDashboard::PutNumber("Lift pos Back", Lift_BackPos) * -1; 

    double LiftOut_Backward = Control_PID(L_Pos,
                            Lift_BackPos,
                            &Lift_ErrPrev[E_RobotLiftBack], 
                            &Lift_IntPrev[E_RobotLiftBack],
                            0.005, 0.0, 0.0, //P I D 
                            1, -1,    //P Upper and lower
                            1.0, -0.1,    //I Upper and lower
                            0,0,          //D Upper and lower
                            1, -1); //Out Upper and lower

    if(Lift_BackPos >= L_End)
    {
        _talon6->Set(ControlMode::PercentOutput, 0);
        return true;
    } else {
        _talon6->Set(ControlMode::PercentOutput, LiftOut_Backward * K_RobotType);
        return false;
    }
}

bool AutonDropBackLift(TalonSRX *_talon6,
                        double   L_Pos,
                        double   L_End) 
{
    double Lift_BackPos = _talon6->GetSelectedSensorPosition() * K_RobotType;

    SmartDashboard::PutNumber("Lift pos Back", Lift_BackPos) * -1; 

    double LiftOut_Backward = Control_PID(L_Pos,
                            Lift_BackPos,
                            &Lift_ErrPrev[E_RobotLiftBack], 
                            &Lift_IntPrev[E_RobotLiftBack],
                            0.005, 0.0, 0.0, //P I D 
                            1, -1,    //P Upper and lower
                            1.0, -0.1,    //I Upper and lower
                            0,0,          //D Upper and lower
                            1, -1); //Out Upper and lower

    if(Lift_BackPos <= L_End)
    {
        _talon6->Set(ControlMode::PercentOutput, 0);
        return true;
    } else {
        _talon6->Set(ControlMode::PercentOutput, LiftOut_Backward * K_RobotType);
        return false;
    }
}

bool AutonRaiseForwardLift(TalonSRX *_talon5,
                           double    L_Pos)
{
    double Lift_ForwardPos = _talon5->GetSelectedSensorPosition() * -1; 

    SmartDashboard::PutNumber("Lift pos forward", Lift_ForwardPos);

    double LiftOut_Forward = Control_PID(L_Pos,
                            Lift_ForwardPos,
                            &Lift_ErrPrev[E_RobotLiftForward], 
                            &Lift_IntPrev[E_RobotLiftForward],
                            0.005, 0.0, 0.0, //P I D 
                            1, -1,    //P Upper and lower
                            1.0, -0.1,    //I Upper and lower
                            0,0,          //D Upper and lower
                            1, -1); //Out Upper and lower
    
    if(Lift_ForwardPos >= -150)
    {
        _talon5->Set(ControlMode::PercentOutput, 0);
        return true;
    } else {
        _talon5->Set(ControlMode::PercentOutput, LiftOut_Forward * -1);
        return false;
    }
}

bool AutonRaiseForwardDrop(TalonSRX *_talon5,
                           double    L_Pos,
                           double    L_End)
{
    double Lift_ForwardPos = _talon5->GetSelectedSensorPosition() * -1; 

    SmartDashboard::PutNumber("Lift pos forward", Lift_ForwardPos);

    double LiftOut_Forward = Control_PID(L_Pos,
                            Lift_ForwardPos,
                            &Lift_ErrPrev[E_RobotLiftForward], 
                            &Lift_IntPrev[E_RobotLiftForward],
                            0.005, 0.0, 0.0, //P I D 
                            1, -1,    //P Upper and lower
                            1.0, -0.1,    //I Upper and lower
                            0,0,          //D Upper and lower
                            1, -1); //Out Upper and lower
    
    if(Lift_ForwardPos <= L_End)
    {
        _talon5->Set(ControlMode::PercentOutput, 0);
        return true;
    } else {
        _talon5->Set(ControlMode::PercentOutput, LiftOut_Forward * -1);
        return false;
    }
}

bool AutonMoveBackLift(TalonSRX *_talon6) 
{
    double Lift_BackPos = _talon6->GetSelectedSensorPosition() * K_RobotType;

    SmartDashboard::PutNumber("Lift pos Back", Lift_BackPos) * -1; 

    double Lift_Desired = 0;
    double LiftOut_Backward = Control_PID(Lift_Desired,
                            Lift_BackPos,
                            &Lift_ErrPrev[E_RobotLiftBack], 
                            &Lift_IntPrev[E_RobotLiftBack],
                            0.005, 0.0, 0.0, //P I D 
                            1, -1,    //P Upper and lower
                            1.0, -0.1,    //I Upper and lower
                            0,0,          //D Upper and lower
                            1, -1); //Out Upper and lower

    if(Lift_BackPos >= -150)
    {
        _talon6->Set(ControlMode::PercentOutput, 0);
        return true;
    } else {
        _talon6->Set(ControlMode::PercentOutput, LiftOut_Backward);
        return false;
    }
}

bool MaintainBackLift(TalonSRX *_talon6, double L_Pos)
{
    double Lift_BackPos = _talon6->GetSelectedSensorPosition() * K_RobotType;

    SmartDashboard::PutNumber("Lift pos Back", Lift_BackPos); 

    double LiftOut_Backward = Control_PID(L_Pos,
                            Lift_BackPos,
                            &Lift_ErrPrev[E_RobotLiftBack], 
                            &Lift_IntPrev[E_RobotLiftBack],
                            0.0002, 0.0003, 0.0, //P I D 
                            0.75, -0.75,    //P Upper and lower
                            1.0, -0.1,    //I Upper and lower
                            0,0,          //D Upper and lower
                            1, -1); //Out Upper and lower

    _talon6->Set(ControlMode::PercentOutput, LiftOut_Backward * K_RobotType);
}

bool MaintainForwardLift(TalonSRX *_talon5, double L_Pos)
{
    double Lift_BackPos = _talon5->GetSelectedSensorPosition() * -1;

    SmartDashboard::PutNumber("Lift pos forward", Lift_BackPos); 

    double LiftOut_Backward = Control_PID(L_Pos,
                            Lift_BackPos,
                            &Lift_ErrPrev[E_RobotLiftForward], 
                            &Lift_IntPrev[E_RobotLiftForward],
                            0.0004, 0.0003, 0.0, //P I D 
                            0.75, -0.75,    //P Upper and lower
                            1.0, -0.1,    //I Upper and lower
                            0,0,          //D Upper and lower
                            1, -1); //Out Upper and lower

    _talon5->Set(ControlMode::PercentOutput, LiftOut_Backward * -1);
}

