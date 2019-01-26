/*
  Calibrations.hpp

   Created on: Feb 14, 2018
       Author: 5561
 */
#ifndef SRC_ROBORIO2018_CALIBRATIONS_HPP_
#define SRC_ROBORIO2018_CALIBRATIONS_HPP_


/* Characterization calibrations:
 * These calibrations set the values that should be fixed (i.e. encoder pulses to revolutions).*/

const int    K_SlotIdx = 0;
const int    K_PIDLoopIdx = 0;
const int    K_TimeoutMs = 10;

/* K_WheelPulseToRev: Number of encoder pulses to a revolution of the wheel.  This is common to both sides of the bot.
                      (pulses / rev)  */
const double K_WheelPulseToRev = 12.75;

/* K_IntakeArmPulseToRev: Number of encoder pulses to relative angle position of the intake arms.  The encoders are
                          non-directional and as a result, additional code is used to determine direction.  As a result,
                          the calibration has been expanded to allow for different pulses per direction.  However, in
                          the 2018 competition bot, this doesn't work due to noise in the signal (need to reevaluate in
                          the future). (pulses / degree) */
const double K_IntakeArmPulseToRev[E_ArmCmndSz] =
    {   0.0,                                 //   E_ArmCmndOff
        0.00557103064066852367688022284123,  //   E_ArmCmndUp
        0.00557103064066852367688022284123}; //   E_ArmCmndDwn

/* K_IntakePulseToTravel: Conversion constant of the number of pulses to the height of the intake lift position
                          (inches / pulse) */
const double K_IntakePulseToTravel = -0.002693208430913349;

/* K_MaxIntakeLiftHeight: Maximum height of the intake lift mechanism.  This will limit the height of the intake.
                          This should be calibrated to be just under the mechanical height limit.  If calibrated higher,
                          there is a risk of brown outs.  (inches) */
const double K_MaxIntakeLiftHeight = 52;

/* K_IntakeLiftLagFilter: Simple first order lag filter associated with the intake lift position.  Used to help filter
                          out noise in the measured position. Range of 0 - 1. (unitless) */
const double K_IntakeLiftLagFilter = 0.8;

/* K_Intake_PID_Gain: PID gains for the intake lift mechanism. Should range from 0 -1 (but typically << 1). */
const double K_Intake_PID_Gain[E_PID_Sz] =
    // P    I      D
    { 0.3, 0.009, 0.0 };

/* K_Intake_PID_Limit: Limits for the intake PID control.  Should be between 0 - 1. */
const double K_Intake_PID_Limit[E_PID_Sz] =
    // P        I      D
    { 0.9, 0.9, 0.0 };
const double K_IntakeDerivativeLimit = 0.0;
const double K_IntakeCmndLimit = 1.0;
const double K_IntakeMinCmndHeight = 2.0;


const double K_HookPulseToTravel = 0.0013329068031563234; // travel (in) / pulse
const double K_MaxHookLiftHeight = 31.05; // Max lift height in inches...
const double K_Hook_PID_Gain[E_PID_Sz] =
    // P        I      D
    { 0.0002, 0.0008, 0.0 };
const double K_Hook_PID_Limit[E_PID_Sz] =
    // P        I      D
    { 0.5, 0.8, 0.0 };
const double K_HookCmndLimit = 0.7;

const double K_Winch = 1.0;
const double K_LukeStopperRamp = 5561;

const double K_RotateGain = 0.75;
const double K_JoystickAnalogDeadband    = 0.1;

const double K_WheelSpeedPID_Gain[E_RobotSideSz][E_PID_Sz] = {
    // P    I    D
    { 0.002,   0.0005,  0.0 }, //LEFT
    { 0.0014,  0.00045, 0.0 }}; // Other Left

//const double K_WheelSpeedPID_GainAuton[E_RobotSideSz][E_PID_Sz] = {
//    // P    I    D
//    { 0.002,   0.00050, 0.0 },  //LEFT
//    { 0.0014,  0.00045, 0.0 }}; //RIGHT

const double K_WheelSpeedPID_GainAuton[E_RobotSideSz][E_PID_Sz] = {
    // P         I        D
    { 0.0035,   0.00020, 0.0 },  //LEFT
    { 0.0025,   0.00020, 0.0 }}; //RIGHT

const double K_WheelSpeedPID_GainRotateAuton[E_RobotSideSz][E_PID_Sz] = {
    // P    I    D
    { 0.0015,   0.0008, 0.0 },  //LEFT
    { 0.0015,   0.0008, 0.0 }}; //RIGHT

const double K_WheelSpeedProportionalLimit[E_RobotSideSz][E_IntergalLimitSz] = {
    // UPPER LOWER
    { 0.7, -0.7}, //LEFT
    { 0.7, -0.7 }  //RIGHT
};

const double K_WheelSpeedIntergalLimit[E_RobotSideSz][E_IntergalLimitSz] = {
    // UPPER LOWER
    { 1.0, -1.0}, //LEFT
    { 1.0, -1.0}  //RIGHT
};

const double K_WheelSpeedDerivativeLimit[E_RobotSideSz][E_IntergalLimitSz] = {
    // UPPER LOWER
    { 1.0, -1.0}, //LEFT
    { 1.0, -1.0}  //RIGHT
};

const double K_WheelSpeedCmndLimit[E_RobotSideSz][E_IntergalLimitSz] = {
    // UPPER LOWER
    { 1.0, -1.0}, //LEFT
    { 1.0, -1.0}  //RIGHT
};

const double K_WheelSpeedLagFilterGain[E_RobotSideSz] =
    { 0.9, 0.9};

const double K_UltraSonicLagFilterGain[E_RobotSideSz] =
    { 0.6, 0.6};

/* K_UltraMaxDistance: Max distance that the ultra sonic distance measurement will show. (inches) */
const double K_UltraMaxDistance = 20.0;

/* K_IntakeRollers: Open loop power command for the intake rollers that is used in auton. (unitless, 0-1) */
const double K_IntakeRollers        =   0.55;

/* K_EndMatchWarningTime: This is the amount of time till the end of the game at which the LED lights will begin to
                          indicate to the driver that the game is about to end.  Typically this is 30 seconds. (seconds)*/
const double K_EndMatchWarningTime  =  30.0;

/* K_WinchOnThreshold: Threshold at which the winch is considered to be on.  This is used to switch the LEDs to an end
                       game configuration. There is a timer associated with this that must expire prior to the lights
                       switching. (unitless, 0-1) */
const double K_WinchOnThreshold     =   0.1;

/* K_LED_WinchOnTime: The amount of time that the winch must be commanded on at the end game before the LEDs will switch
                      over. (seconds)  */
const double K_LED_WinchOnTime      =   3.0;

const double K_LiftSpeedDropGain = 0.81;

const double K_IntakeRollerAutoPullIn = 0.05;

const double K_DesiredVerticalSpeedAxis[10] = {-0.9,
                                               -0.7,
                                               -0.5,
                                               -0.3,
                                               -0.1,
                                                0.1,
                                                0.3,
                                                0.5,
                                                0.7,
                                                0.9};

//const double K_DesiredVerticalSpeed[10] = {-30.0,  // -0.9
//                                           -15.0,  // -0.7
//                                           -10.0,  // -0.5
//                                            -2.0,  // -0.3
//                                             0.0,  // -0.1
//                                             0.0,  //  0.1
//                                             2.0,  //  0.3
//                                            10.0,  //  0.5
//                                            15.0,  //  0.7
//                                            30.0}; //  0.9

const double K_DesiredVerticalSpeed[10] = {-50.0,  // -0.9
                                           -15.0,  // -0.7
                                           -10.0,  // -0.5
                                            -2.0,  // -0.3
                                             0.0,  // -0.1
                                             0.0,  //  0.1
                                             2.0,  //  0.3
                                            10.0,  //  0.5
                                            15.0,  //  0.7
                                            50.0}; //  0.9

const double K_DesiredDriveSpeedAxis[20] = {-0.95,
                                            -0.85,
                                            -0.75,
                                            -0.65,
                                            -0.55,
                                            -0.45,
                                            -0.35,
                                            -0.25,
                                            -0.15,
                                            -0.10,
                                             0.10,
                                             0.15,
                                             0.25,
                                             0.35,
                                             0.45,
                                             0.55,
                                             0.65,
                                             0.75,
                                             0.85,
                                             0.95};

// Fast Table
//const double K_DesiredDriveSpeed[20] = {-150.00,  //-0.95
//                                        -131.25,  //-0.85
//                                        -112.50,  //-0.75
//                                         -93.75,  //-0.65
//                                         -75.00,  //-0.55
//                                         -56.25,  //-0.45
//                                         -37.50,  //-0.35
//                                         -18.75,  //-0.25
//                                          -5.00,  //-0.15
//                                           0.00,  //-0.10
//                                           0.00,  // 0.10
//                                           5.00,  // 0.15
//                                          18.75,  // 0.25
//                                          37.50,  // 0.35
//                                          56.25,  // 0.45
//                                          75.00,  // 0.55
//                                          93.75,  // 0.65
//                                         112.50,  // 0.75
//                                         131.25,  // 0.85
//                                         150.00}; // 0.95

// Slow Table
const double K_DesiredDriveSpeed[20] = {-150.00,  //-0.95
                                        -111.56,  //-0.85
                                         -84.375, //-0.75
                                         -56.25,//-0.65
                                         -37.5,  //-0.55
                                         -28.125, //-0.45
                                         -18.75,  //-0.35
                                          -9.375,  //-0.25
                                          -2.50,  //-0.15
                                           0.00,  //-0.10
                                           0.00,  // 0.10
                                           2.50,  // 0.15
                                           9.375,  // 0.25
                                          18.75,  // 0.35
                                          28.125,  // 0.45
                                          37.5,  // 0.55
                                          56.25,  // 0.65
                                          84.375,  // 0.75
                                         111.56,  // 0.85
                                         150.00}; // 0.95

/* K_AutonDebounceThreshold: Seconds of debounce time spent after a task is
 *                           completed in auton to ensure that the robot has
 *                           completed the task*/
const double K_AutonDebounceThreshold     =  0.05;

const double K_AutonRotateAnglePropGx     =   0.65561;  // RPM/Degrees
const double K_AutonRotateAngleDeadband   =   2.0;  // Degrees
const double K_AutonRotateMaxSpeed        =  28.5561;  // RPM
const double K_AutonRotateMinSpeed        =  11.5561;  // RPM

const double K_AutonDriveDistanceUltraDeadband =  0.5;  // Inches - for the ultrasonic sensor control
const double K_AutonDriveMinSpeedUltra         = 20.5561; // RPM

const double K_AutonDriveDistanceDeadband =   5.0;  // Inches
const double K_AutonDriveDistanceToSlow   =  24.5561;  // Inches
const double K_AutonDriveSpeedRamp        = 100.0;  // RPM/Sec
const double K_AutonDriveMaxSpeed         = 139.5561;  // RPM
const double K_AutonDriveMinSpeed         =  44.5561;  // RPM

const double K_AutonIntakeRamp             =  60.0;  // Inch/Sec
const double K_AutonIntakeDistanceDeadband =   2.0;  // Inches

/* K_AutonIntakeAngleCmnd: Percent of possible motor controller output to the
                           motors controlling the arm angle while in auton.  */
const double K_AutonIntakeAngleCmnd        =   0.6;

/* K_AutonCommand: This table contains all of the possible commands that could be expected in Auton.
                   These will go sequentially go through the possible combinations.  You can have two
                   actuators being controlled at once when it makes sense.  For example, you can only
                   control the drive wheels with a single command at one time (so you couldn't
                   command a rotation and a drive command at the same time). The following are the
                   units for the various actuators:

                   Encoder:    Drive distance in inches using encoders
                   ArmAng:     Open loop power command for the given time
                   Rollers:    Open loop power command for the given time
                   Lift:       Lift position of intake mechanism in inches
                   Rotate:     Rotation of the bot in degrees.
                   UltraSonic: Drive distance in inches using the ultrasonic sensors from the sensor*/
const AutonControlPlan K_AutonCommand[E_AutonOptSz] =
    {
      { // E_AutonOpt0 - Left- side switch - left
        {   E_ActuatorDriveEncoder,          E_ActuatorRotate,            E_ActuatorLift, E_ActuatorDriveUltraSonic,         E_ActuatorRollers,    E_ActuatorDriveEncoder,          E_ActuatorRotate,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                      155,                        90,                        20,                         4,                       1.5,                       -10,                       -90,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,       E_ActuatorArmAngDwn,            E_ActuatorNone,       E_ActuatorArmAngDwn,            E_ActuatorNone,        E_ActuatorArmAngUp,            E_ActuatorLift,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                      0.15,                         0,                       0.6,                         0,                       0.6,                         5,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt1 - Left - Scale - Left
        {   E_ActuatorDriveEncoder,          E_ActuatorRotate,    E_ActuatorDriveEncoder,       E_ActuatorArmAngDwn,         E_ActuatorRollers,        E_ActuatorArmAngUp,          E_ActuatorRotate,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                      300,                        90,                         2,                      0.35,                       1.5,                      0.35,                       -90,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {      E_ActuatorArmAngDwn,            E_ActuatorNone,            E_ActuatorLift,            E_ActuatorNone,            E_ActuatorNone,    E_ActuatorDriveEncoder,            E_ActuatorLift,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                     0.15,                         0,                        52,                         0,                         0,                       -10,                         5,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt2 - middle - front switch - left
        {   E_ActuatorDriveEncoder,          E_ActuatorRotate,    E_ActuatorDriveEncoder,          E_ActuatorRotate, E_ActuatorDriveUltraSonic,       E_ActuatorArmAngDwn,          E_ActuatorRollers,       E_ActuatorArmAngUp,    E_ActuatorDriveEncoder,          E_ActuatorRotate,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                  14.5561,                  -47.5561,                       124,                   50.5561,                         4,                       0.6,                        1.5,                      0.6,                       -10,                      -180,                         0,                         0,                         0,                         0,                         0},
        {      E_ActuatorArmAngDwn,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorLift,            E_ActuatorNone,             E_ActuatorNone,           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorLift,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                     0.15,                         0,                         0,                         0,                        20,                         0,                          0,                        0,                         0,                         5,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt3 - middle - front switch - right
        {   E_ActuatorDriveEncoder,          E_ActuatorRotate,    E_ActuatorDriveEncoder, E_ActuatorDriveUltraSonic,       E_ActuatorArmAngDwn,         E_ActuatorRollers,         E_ActuatorArmAngUp,   E_ActuatorDriveEncoder,          E_ActuatorRotate,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                       10,                         18,                        75,                        4,                       0.6,                       1.5,                        0.6,                      -10,                      -180,                         0,                         0,                         0,                         0,                         0,                         0},
        {      E_ActuatorArmAngDwn,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorLift,            E_ActuatorNone,            E_ActuatorNone,             E_ActuatorNone,           E_ActuatorNone,            E_ActuatorLift,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                     0.15,                         0,                         0,                        20,                         0,                         0,                          0,                        0,                         5,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt4 - Right - Scale - Right
        {   E_ActuatorDriveEncoder,          E_ActuatorRotate,    E_ActuatorDriveEncoder,       E_ActuatorArmAngDwn,         E_ActuatorRollers,        E_ActuatorArmAngUp,          E_ActuatorRotate,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                      300,                       -90,                         2,                      0.35,                       1.5,                      0.35,                        90,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {      E_ActuatorArmAngDwn,            E_ActuatorNone,            E_ActuatorLift,            E_ActuatorNone,            E_ActuatorNone,    E_ActuatorDriveEncoder,            E_ActuatorLift,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                     0.15,                         0,                        52,                         0,                         0,                       -10,                         5,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt5 - right - switch - right
        {   E_ActuatorDriveEncoder,          E_ActuatorRotate,            E_ActuatorLift, E_ActuatorDriveUltraSonic,         E_ActuatorRollers,    E_ActuatorDriveEncoder,          E_ActuatorRotate,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                      155,                       -90,                        20,                         4,                       1.5,                        -5,                        90,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,       E_ActuatorArmAngDwn,            E_ActuatorNone,       E_ActuatorArmAngDwn,            E_ActuatorNone,        E_ActuatorArmAngUp,            E_ActuatorLift,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                      0.15,                         0,                       0.6,                         0,                       0.6,                         5,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      },
      { // E_AutonOpt6 - Default
        {   E_ActuatorDriveEncoder,       E_ActuatorArmAngDwn,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                       90,                      0.15,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0},
        {           E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone,            E_ActuatorNone},
        {                        0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0,                         0}
      }
    };
#endif /* SRC_ROBORIO2018_CALIBRATIONS_HPP_ */
