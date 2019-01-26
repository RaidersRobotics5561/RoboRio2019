/************************************************************************************************************
*
*  Robot base code for RoboRio
*
*  Flushing Raiders Robotics - Team 5561
*
*  Robot code for the 2018 season.
*
************************************************************************************************************/

/************************************************************************************************************
* Header files
************************************************************************************************************/
#include "const.h"
#include "LookUp.hpp"
#include "Calibrations.hpp"
#include "Control_PID.hpp"
#include "VariableInit.hpp"
#include "Output_SmartDashboard.hpp"
#include "SignalFilter.hpp"
#include "Vars.hpp"
#include "AnalogTriggerType.h"
#include "Output_Actuators.hpp"
#include "Input_Controller.hpp"
#include "Control_LED.h"
#include "Control_Auton.hpp"


/************************************************************************************************************
* Header files
************************************************************************************************************/
double V_EndGameWinchTime;
bool   V_LED_RainbowLatch;

double V_HookPosition;
double V_HookPositionErrorPrev;
double V_HookPositionErrorIntegral;
double V_HookLiftHeightDesired;
double V_HookPID_Gain[E_PID_Sz];

double V_IntakePosition;
double V_IntakePositionErrorPrev;
double V_IntakePositionErrorIntegral;
double V_IntakeLiftHeightDesired;
double V_IntakePID_Gain[E_PID_Sz];
double V_IntakePositionPrev;

double V_IntakeArmPulseToRev[E_ArmCmndSz];

double V_WheelRPM_Filt[E_RobotSideSz];
double V_WheelRPM_FiltPrev[E_RobotSideSz];
double V_WheelRPM_Desired[E_RobotSideSz];
double V_WheelSpeedErrorPrev[E_RobotSideSz];
double V_WheelSpeedErrorIntegral[E_RobotSideSz];
double V_WheelRPM_Raw[E_RobotSideSz];
double V_WheelProportionalGain[E_RobotSideSz];
double V_WheelIntegralGain[E_RobotSideSz];
double V_WheelDerivativeGain[E_RobotSideSz];
double V_WheelSpeedLagFiltGain[E_RobotSideSz];
double V_DistanceTraveled[E_RobotSideSz];
double V_DistanceTraveledAvg;
double V_Revolutions[E_RobotSideSz];
double V_LukeStopperRamp;
double V_RotateGain;

double V_UltraSonicDistance[E_RobotSideSz]; // Measured, filtered ultrasonic distance

T_RobotSide       V_AutonTargetSide[3];
T_AutonStartPos   V_AutonStartPos;
T_AutonPreference V_AutonPreferenceFinal;
double            V_AutonWheelDebounceTimer[E_RobotSideSz];
double            V_AutonRotateDebounceTimer;
double            V_AutonIntakeLiftDebounceTimer;

T_RoboState V_RobatState;
T_DriveMode DriveMode;

double V_GyroAngleRelative; // This is the relative angle of the robot, since the last time the variable clear function was called.
double V_GyroAngleOffset;   // This is the amount of drift and/or change that the gyro has observed since the robot was started.
double V_ArmAngleDeg;

double V_RobotUserCmndPct[E_RobotUserCmndSz]; // This is the requested value from the driver for the various motors/functions
double V_RobotMotorCmndPct[E_RobotMotorSz];

class Robot: public IterativeRobot {

	frc::SendableChooser<std::string> V_StartingPosition;
	const std::string C_StartOpt0 = "Default";
	const std::string C_StartOpt1 = "Left";
	const std::string C_StartOpt2 = "Middle";
  const std::string C_StartOpt3 = "Right";
	std::string V_StartOptSelected;

  frc::SendableChooser<std::string> V_AutonPreference;
  const std::string C_Preference0 = "Switch";
  const std::string C_Preference1 = "Scale";
  std::string V_AutonPreferenceSelected;

private:
	//left Back, SRX:left Front #1
	TalonSRX * _talon0 = new TalonSRX(1);
	//left Front, SRX:left Back #2
	TalonSRX * _talon1 = new TalonSRX(2);
	//right Front, SRX:Right Left #3
	TalonSRX * _talon2 = new TalonSRX(3);
	//right Back, SRX:Right Right #4
	TalonSRX * _talon3 = new TalonSRX(4);
	//Intake #5
	TalonSRX * _talon4 = new TalonSRX(5);
	//Hook delivery #6
	TalonSRX * _talon5 = new TalonSRX(6);
	// Left articulation
	Spark *Spark1 = new Spark(0);
	// Left intake
	Spark *Spark2 = new Spark(1);
	//Right intake
	Talon *Talon_PWM0 = new Talon(2);
	//Right articulation
	Talon *Talon_PWM1 = new Talon(3);
	//Winch
	Talon *Talon_PWM2 = new Talon(4);

	AnalogTrigger *mAnalogTrigger = new AnalogTrigger(4); // create an encoder pulse trigger
	Counter* mCounter; // count the encoder pulse triggers in current direction

	DigitalOutput *V_LED_State0 = new DigitalOutput(0);
	DigitalOutput *V_LED_State1 = new DigitalOutput(1);
	DigitalOutput *V_LED_State2 = new DigitalOutput(2);
	DigitalOutput *V_LED_State3 = new DigitalOutput(3);

  Ultrasonic *_UltraLeft;  // creates the ultra sonic range finder left object
  Ultrasonic *_UltraRight; // creates the ultra sonic range finder right object

	ADXRS450_Gyro Gyro;

	Preferences *Prefs;

	Joystick *_joy1 = new Joystick(0);
	Joystick *_joy2 = new Joystick(1);

	std::string gameData;


/******************************************************************************
 * Function:     DisabledInit
 *
 * Description: Initialization when the robot enters the disabled state.
 *
 ******************************************************************************/
void DisabledInit()
  {
  V_RobatState = E_Disabled;

//  VariableInit(Prefs,
//               mCounter,
//               _talon0,
//               _talon1,
//               _talon2,
//               _talon3,
//               _talon4,
//               _talon5,
//               &Gyro);

  UpdateLED_Output(V_RobatState,
                   false,
                   V_RobotMotorCmndPct[E_RobotMotorWinch],
                   E_RobotSideSz,
                   V_LED_State0,
                   V_LED_State1,
                   V_LED_State2,
                   V_LED_State3);

  UpdateActuatorCmnds(_talon0,
                      _talon1,
                      _talon2,
                      _talon3,
                      _talon4,
                      _talon5,
                      Spark1,
                      Spark2,
                      Talon_PWM0,
                      Talon_PWM1,
                      Talon_PWM2,
                      V_RobotMotorCmndPct);

  UpdateSmartDashboad();
  }


/******************************************************************************
 * Function:     DisabledPeriodic
 *
 * Description:
 *
 ******************************************************************************/
void DisabledPeriodic()
  {
  V_RobatState = E_Disabled;

//  VariableInit(Prefs,
//               mCounter,
//               _talon0,
//               _talon1,
//               _talon2,
//               _talon3,
//               _talon4,
//               _talon5,
//               &Gyro);

  while (IsDisabled())
    {
    UpdateLED_Output(V_RobatState,
                     false,
                     V_RobotMotorCmndPct[E_RobotMotorWinch],
                     E_RobotSideSz,
                     V_LED_State0,
                     V_LED_State1,
                     V_LED_State2,
                     V_LED_State3);

    UpdateActuatorCmnds(_talon0,
                        _talon1,
                        _talon2,
                        _talon3,
                        _talon4,
                        _talon5,
                        Spark1,
                        Spark2,
                        Talon_PWM0,
                        Talon_PWM1,
                        Talon_PWM2,
                        V_RobotMotorCmndPct);

    UpdateSmartDashboad();

    Wait(C_ExeTime);
    }
  }


/******************************************************************************
 * Function:     RobotInit
 *
 * Description:
 *
 ******************************************************************************/
void RobotInit()
  {

	CameraServer::GetInstance()->StartAutomaticCapture(0);
	CameraServer::GetInstance()->StartAutomaticCapture(1);

	V_StartingPosition.AddDefault(C_StartOpt0, C_StartOpt0);
	V_StartingPosition.AddObject(C_StartOpt1, C_StartOpt1);
	V_StartingPosition.AddObject(C_StartOpt2, C_StartOpt2);
	V_StartingPosition.AddObject(C_StartOpt3, C_StartOpt3);
	frc::SmartDashboard::PutData("Starting Position", &V_StartingPosition);

	V_AutonPreference.AddDefault(C_Preference0, C_Preference0);
	V_AutonPreference.AddObject(C_Preference1, C_Preference1);
  frc::SmartDashboard::PutData("Switch Scale Preference", &V_AutonPreference);

	Prefs = Preferences::GetInstance();

	V_RobatState = E_Disabled;

	mAnalogTrigger->SetLimitsVoltage(3.5, 3.8); // values higher than the highest minimum (pulse floor), lower than the lowest maximum (pulse ceiling)
	mCounter = new Counter(4);

	_UltraLeft  = new Ultrasonic(7, 6); // assigns ultra to be an ultrasonic sensor which uses DigitalOutput 1 for the echo pulse and DigitalInput 1 for the trigger pulse
	_UltraLeft->SetAutomaticMode(true); // turns on automatic mode

	_UltraRight = new Ultrasonic(9, 8); // assigns ultra to be an ultrasonic sensor which uses DigitalOutput 1 for the echo pulse and DigitalInput 1 for the trigger pulse
	_UltraRight->SetAutomaticMode(true); // turns on automatic mode

	V_LED_State0->Set(false);
	V_LED_State1->Set(false);
	V_LED_State2->Set(false);
	V_LED_State3->Set(false);

	Gyro.Calibrate();

	_talon0->ConfigSelectedFeedbackSensor(
			FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

	_talon2->ConfigSelectedFeedbackSensor(
			FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

	_talon3->ConfigSelectedFeedbackSensor(
			FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

	_talon4->ConfigSelectedFeedbackSensor(
			FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

	_talon5->ConfigSelectedFeedbackSensor(
			FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

	_talon0->SetSensorPhase(true);
	_talon2->SetSensorPhase(true);
	_talon3->SetSensorPhase(true);
	_talon4->SetSensorPhase(true);
	_talon5->SetSensorPhase(true);

	/* set the peak and nominal outputs, 12V means full */

	_talon0->ConfigNominalOutputForward(0, K_TimeoutMs);
	_talon1->ConfigNominalOutputForward(0, K_TimeoutMs);
	_talon2->ConfigNominalOutputForward(0, K_TimeoutMs);
	_talon3->ConfigNominalOutputForward(0, K_TimeoutMs);
	_talon4->ConfigNominalOutputForward(0, K_TimeoutMs);
	_talon5->ConfigNominalOutputForward(0, K_TimeoutMs);

	_talon0->ConfigNominalOutputReverse(0, K_TimeoutMs);
	_talon1->ConfigNominalOutputReverse(0, K_TimeoutMs);
	_talon2->ConfigNominalOutputReverse(0, K_TimeoutMs);
	_talon3->ConfigNominalOutputReverse(0, K_TimeoutMs);
	_talon4->ConfigNominalOutputReverse(0, K_TimeoutMs);
	_talon5->ConfigNominalOutputReverse(0, K_TimeoutMs);

	_talon0->ConfigPeakOutputForward(1, K_TimeoutMs);
	_talon1->ConfigPeakOutputForward(1, K_TimeoutMs);
	_talon2->ConfigPeakOutputForward(1, K_TimeoutMs);
	_talon3->ConfigPeakOutputForward(1, K_TimeoutMs);
	_talon4->ConfigPeakOutputForward(1, K_TimeoutMs);
	_talon5->ConfigPeakOutputForward(1, K_TimeoutMs);

	_talon0->ConfigPeakOutputReverse(-1, K_TimeoutMs);
	_talon1->ConfigPeakOutputReverse(-1, K_TimeoutMs);
	_talon2->ConfigPeakOutputReverse(-1, K_TimeoutMs);
	_talon3->ConfigPeakOutputReverse(-1, K_TimeoutMs);
	_talon4->ConfigPeakOutputReverse(-1, K_TimeoutMs);
	_talon5->ConfigPeakOutputReverse(-1, K_TimeoutMs);

	_talon4->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
	_talon5->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);

  V_IntakePosition = 0.0;
  V_IntakePositionPrev = 0.0;
  V_IntakeLiftHeightDesired = 0.0;

   VariableInit(Prefs,
                mCounter,
                _talon0,
                _talon1,
                _talon2,
                _talon3,
                _talon4,
                _talon5,
                &Gyro);
  }


/******************************************************************************
 * Function:     TeleopPeriodic
 *
 * Description:  RoboRio call during the teleop periodic period.
 ******************************************************************************/
	void TeleopPeriodic()
	  {
    T_ArmCmnd L_ArmAnglePrev     = E_ArmCmndOff;
    T_ArmCmnd L_ArmAnglePrevPrev = E_ArmCmndOff;

//		VariableInit(Prefs,
//		             mCounter,
//		             _talon0,
//		             _talon1,
//		             _talon2,
//		             _talon3,
//		             _talon4,
//		             _talon5,
//		             &Gyro);

    V_RobatState = E_Teleop;

		while (IsOperatorControl() && IsEnabled()) {

    DtrmnControllerMapping(_joy1,
                           _joy2);

    /* Read all of the available sensors */
    Read_Sensors(_talon0,
                 _talon2,
                 _talon4,
                 _talon5,
                 mCounter,
                 &Gyro,
                 &V_ArmAngleDeg,
                 L_ArmAnglePrev,
                 L_ArmAnglePrevPrev,
                 _UltraLeft,
                 _UltraRight);

    V_WheelRPM_Desired[E_RobotSideLeft]  = DesiredSpeed(V_RobotUserCmndPct[E_RobotUserCmndLeftWheel],
                                                        V_WheelRPM_Filt[E_RobotSideLeft]);

    V_WheelRPM_Desired[E_RobotSideRight] = DesiredSpeed(V_RobotUserCmndPct[E_RobotUserCmndRightWheel],
                                                        V_WheelRPM_Filt[E_RobotSideRight]);

    V_RobotMotorCmndPct[E_RobotMotorLeftWheel]  = Control_PID(V_WheelRPM_Desired[E_RobotSideLeft],
                                                              V_WheelRPM_Filt[E_RobotSideLeft],
                                                              &V_WheelSpeedErrorPrev[E_RobotSideLeft],
                                                              &V_WheelSpeedErrorIntegral[E_RobotSideLeft],
                                                              V_WheelProportionalGain[E_RobotSideLeft],
                                                              V_WheelIntegralGain[E_RobotSideLeft],
                                                              V_WheelDerivativeGain[E_RobotSideLeft],
                                                              K_WheelSpeedProportionalLimit[E_RobotSideLeft][E_IntergalUpperLimit],
                                                              K_WheelSpeedProportionalLimit[E_RobotSideLeft][E_IntergalLowerLimit],
                                                              K_WheelSpeedIntergalLimit[E_RobotSideLeft][E_IntergalUpperLimit],
                                                              K_WheelSpeedIntergalLimit[E_RobotSideLeft][E_IntergalLowerLimit],
                                                              K_WheelSpeedDerivativeLimit[E_RobotSideLeft][E_IntergalUpperLimit],
                                                              K_WheelSpeedDerivativeLimit[E_RobotSideLeft][E_IntergalLowerLimit],
                                                              K_WheelSpeedCmndLimit[E_RobotSideLeft][E_IntergalUpperLimit],
                                                              K_WheelSpeedCmndLimit[E_RobotSideLeft][E_IntergalLowerLimit]);

    V_RobotMotorCmndPct[E_RobotMotorRightWheel] = Control_PID(V_WheelRPM_Desired[E_RobotSideRight],
                                                              V_WheelRPM_Filt[E_RobotSideRight],
                                                              &V_WheelSpeedErrorPrev[E_RobotSideRight],
                                                              &V_WheelSpeedErrorIntegral[E_RobotSideRight],
                                                              V_WheelProportionalGain[E_RobotSideRight],
                                                              V_WheelIntegralGain[E_RobotSideRight],
                                                              V_WheelDerivativeGain[E_RobotSideRight],
                                                              K_WheelSpeedProportionalLimit[E_RobotSideRight][E_IntergalUpperLimit],
                                                              K_WheelSpeedProportionalLimit[E_RobotSideRight][E_IntergalLowerLimit],
                                                              K_WheelSpeedIntergalLimit[E_RobotSideRight][E_IntergalUpperLimit],
                                                              K_WheelSpeedIntergalLimit[E_RobotSideRight][E_IntergalLowerLimit],
                                                              K_WheelSpeedDerivativeLimit[E_RobotSideRight][E_IntergalUpperLimit],
                                                              K_WheelSpeedDerivativeLimit[E_RobotSideRight][E_IntergalLowerLimit],
                                                              K_WheelSpeedCmndLimit[E_RobotSideRight][E_IntergalUpperLimit],
                                                              K_WheelSpeedCmndLimit[E_RobotSideRight][E_IntergalLowerLimit]);

    V_IntakeLiftHeightDesired =  DesiredLiftHeight(V_RobotUserCmndPct[E_RobotUserCmndLift],
                                                   V_IntakeLiftHeightDesired,
                                                   K_MaxIntakeLiftHeight);

    V_RobotMotorCmndPct[E_RobotMotorLift]  = Control_PID( V_IntakeLiftHeightDesired,
                                                          V_IntakePosition,
                                                         &V_IntakePositionErrorPrev,
                                                         &V_IntakePositionErrorIntegral,
                                                          V_IntakePID_Gain[E_PID_Proportional],
                                                          V_IntakePID_Gain[E_PID_Integral],
                                                          V_IntakePID_Gain[E_PID_Derivative],
                                                          K_Intake_PID_Limit[E_PID_Proportional],
                                                         -K_Intake_PID_Limit[E_PID_Proportional],
                                                          K_Intake_PID_Limit[E_PID_Integral],
                                                         -K_Intake_PID_Limit[E_PID_Integral],
                                                          K_Intake_PID_Limit[E_PID_Derivative],
                                                         -K_Intake_PID_Limit[E_PID_Derivative],
                                                          K_IntakeCmndLimit,
                                                         -K_IntakeCmndLimit);

    V_RobotMotorCmndPct[E_RobotMotorLift] = LiftCmdDisable(V_IntakePosition,
                                                           V_IntakeLiftHeightDesired,
                                                           K_IntakeMinCmndHeight,
                                                           V_RobotMotorCmndPct[E_RobotMotorLift]);


    V_RobotMotorCmndPct[E_RobotMotorWinch] = V_RobotUserCmndPct[E_RobotUserCmndWinch]; // climb direction

    V_HookLiftHeightDesired =  DesiredLiftHeight(V_RobotUserCmndPct[E_RobotUserCmndHook],
                                                 V_HookLiftHeightDesired,
                                                 K_MaxHookLiftHeight);

    V_RobotMotorCmndPct[E_RobotMotorHook]  = Control_PID( V_HookLiftHeightDesired,
                                                          V_HookPosition,
                                                         &V_HookPositionErrorPrev,
                                                         &V_HookPositionErrorIntegral,
                                                          V_HookPID_Gain[E_PID_Proportional],
                                                          V_HookPID_Gain[E_PID_Integral],
                                                          V_HookPID_Gain[E_PID_Derivative],
                                                          K_Hook_PID_Limit[E_PID_Proportional],
                                                         -K_Hook_PID_Limit[E_PID_Proportional],
                                                          K_Hook_PID_Limit[E_PID_Integral],
                                                         -K_Hook_PID_Limit[E_PID_Integral],
                                                          K_Hook_PID_Limit[E_PID_Derivative],
                                                         -K_Hook_PID_Limit[E_PID_Derivative],
                                                          K_HookCmndLimit,
                                                         -K_HookCmndLimit);

    V_RobotMotorCmndPct[E_RobotMotorIntakeArmAng] = V_RobotUserCmndPct[E_RobotUserCmndIntakeArmAng];

	  V_RobotMotorCmndPct[E_RobotMotorIntakeRoller] = V_RobotUserCmndPct[E_RobotUserCmndIntakeRoller];

//    L_ArmAnglePrevPrev = L_ArmAnglePrev;
//    L_ArmAnglePrev = E_ArmCmndOff;
//
//    if (V_RobotMotorCmndPct[E_RobotMotorIntakeArmAng] > 0)
//      {
//      L_ArmAnglePrev = E_ArmCmndUp;
//      }
//    else if (V_RobotMotorCmndPct[E_RobotMotorIntakeArmAng] < 0)
//      {
//      L_ArmAnglePrev = E_ArmCmndDwn;
//      }

    UpdateLED_Output(V_RobatState,
                     false,
                     V_RobotMotorCmndPct[E_RobotMotorWinch],
                     E_RobotSideSz,
                     V_LED_State0,
                     V_LED_State1,
                     V_LED_State2,
                     V_LED_State3);

	  UpdateActuatorCmnds(_talon0,
	                      _talon1,
	                      _talon2,
	                      _talon3,
	                      _talon4,
	                      _talon5,
	                      Spark1,
	                      Spark2,
	                      Talon_PWM0,
	                      Talon_PWM1,
	                      Talon_PWM2,
	                      V_RobotMotorCmndPct);

		UpdateSmartDashboad();

		Wait(C_ExeTime);
		}
	}

/******************************************************************************
 * Function:     AutonomousInit
 *
 * Description: This method is called once each time the robot enters Autonomous
 *
 ******************************************************************************/
void AutonomousInit()
  {


//	_talon0->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
//	_talon3->SetSelectedSensorPosition(0, K_SlotIdx, K_TimeoutMs);
//
//  VariableInit(Prefs,
//               mCounter,
//               _talon0,
//               _talon1,
//               _talon2,
//               _talon3,
//               _talon4,
//               _talon5,
//               &Gyro);


  }

/******************************************************************************
 * Function:     AutonomousPeriodic
 *
 * Description:
 *
 ******************************************************************************/
void AutonomousPeriodic()
  {
  int L_Index;
  T_AutonOpt       L_AutonOption           = E_AutonOpt0;
  T_Actuator       L_CntrlActuator         = E_ActuatorNone;
  T_AutonCntrlType L_AutonCntrlType        = E_AutonCntrlPrimary;
  T_AutonStep      L_AutonStep             = E_AutonStep0;
  double           L_AutonTarget           = 0.0;
  bool             L_ControlComplete[E_AutonCntrlSz];
  T_RobotMotor     L_RobotMotor            = E_RobotMotorLeftWheel;
  T_RobotSide      L_TurnSignal            = E_RobotSideSz;

  VariableInit(Prefs,
               mCounter,
               _talon0,
               _talon1,
               _talon2,
               _talon3,
               _talon4,
               _talon5,
               &Gyro);

  V_RobatState = E_Auton;

  gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

  if (gameData.length() > 0)
    {
    for (L_Index = 0; L_Index < 3; L_Index++)
      {
      if(gameData[L_Index] == 'L')
        {
        V_AutonTargetSide[L_Index] = E_RobotSideLeft;
        }
      else
        {
        V_AutonTargetSide[L_Index] = E_RobotSideRight;
        }
      }
    }

  V_StartOptSelected = V_StartingPosition.GetSelected();

  if (V_StartOptSelected == C_StartOpt1)
    {
    V_AutonStartPos = E_AutonStartPosLeft;
    }
  else if (V_StartOptSelected == C_StartOpt2)
    {
    V_AutonStartPos = E_AutonStartPosMiddle;
    }
  else if (V_StartOptSelected == C_StartOpt3)
    {
    V_AutonStartPos = E_AutonStartPosRight;
    }
  else
    {
    V_AutonStartPos = E_AutonStartPosDefault;
    }

  V_AutonPreferenceSelected = V_AutonPreference.GetSelected();

  if (V_AutonPreferenceSelected == C_Preference0)
    {
    V_AutonPreferenceFinal = E_AutonPreferenceSwitch;
    }
  else
    {
    V_AutonPreferenceFinal = E_AutonPreferenceScale;
    }

  L_AutonOption =  DtrmnAutonOption(V_AutonTargetSide[0],
                                    V_AutonTargetSide[1],
                                    V_AutonStartPos,
                                    V_AutonPreferenceFinal);

  L_ControlComplete[E_AutonCntrlPrimary]   = false;
  L_ControlComplete[E_AutonCntrlSecondary] = false;

  while (IsAutonomous() && IsEnabled())
    {
    /* Read all of the available sensors */
    Read_Sensors(_talon0,
                 _talon2,
                 _talon4,
                 _talon5,
                 mCounter,
                 &Gyro,
                 &V_ArmAngleDeg,
                 E_ArmCmndOff,
                 E_ArmCmndOff,
                 _UltraLeft,
                 _UltraRight);

    if ((L_ControlComplete[E_AutonCntrlPrimary]   == true) &&
        (L_ControlComplete[E_AutonCntrlSecondary] == true) &&
        (L_AutonStep < E_AutonStepSz))
      {
      /* Ok, we have met the primary and secondary conditions.  Let's reset the auton specific variables and
       * get ready for the next step. */
      L_ControlComplete[E_AutonCntrlPrimary]   = false;
      L_ControlComplete[E_AutonCntrlSecondary] = false;

      AutonVariableInit(Prefs,
                        mCounter,
                        _talon0,
                        _talon1,
                        _talon2,
                        _talon3,
                        _talon4,
                        _talon5,
                        &Gyro);
      /* Ok, let's go to the next step: */
      L_AutonStep = T_AutonStep(int(L_AutonStep) + 1);
      }

    if (L_AutonStep < E_AutonStepSz)
      {
      /* Only allow this if we are within the Auton bounds: */

	  /* Reset the turn signal each loop.  Need to determine if we are turning for LEDs. */
	  L_TurnSignal = E_RobotSideSz;
      for (L_AutonCntrlType = E_AutonCntrlPrimary;
           L_AutonCntrlType < E_AutonCntrlSz;
           L_AutonCntrlType = T_AutonCntrlType(int(L_AutonCntrlType) + 1))
        {
        if (L_AutonCntrlType == E_AutonCntrlPrimary)
          {
          L_CntrlActuator = K_AutonCommand[L_AutonOption].PrimaryActuator[L_AutonStep];
          L_AutonTarget   = K_AutonCommand[L_AutonOption].PrimaryCmndValue[L_AutonStep];
          }
        else
          {
          L_CntrlActuator = K_AutonCommand[L_AutonOption].SecondaryActuator[L_AutonStep];
          L_AutonTarget   = K_AutonCommand[L_AutonOption].SecondaryCmndValue[L_AutonStep];
          }

        switch (L_CntrlActuator)
          {
          case E_ActuatorRotate:          if (L_AutonTarget > 0) {L_TurnSignal = E_RobotSideRight;} else {L_TurnSignal = E_RobotSideLeft;}
          case E_ActuatorDriveEncoder:
          case E_ActuatorDriveUltraSonic: L_ControlComplete[L_AutonCntrlType] = CntrlAutonDrive(L_CntrlActuator,
                                                                                                L_AutonTarget);          break;

          case E_ActuatorLift:            L_ControlComplete[L_AutonCntrlType] = CntrlAutonLift(L_CntrlActuator,
                                                                                               L_AutonTarget);           break;

          case E_ActuatorRollers:
          case E_ActuatorArmAngUp:
          case E_ActuatorArmAngDwn:       L_ControlComplete[L_AutonCntrlType] = CntrlAutonOpenLoopTimer(L_CntrlActuator,
                                                                                                        L_AutonTarget);  break;

          case E_ActuatorNone:
          default:                        L_ControlComplete[L_AutonCntrlType] = true;                                    break;
          }
        }
      }
    else
      {
      /* We have reached the end of the auton program.  Let's make sure all of the commands are zero and hang out till
       * we exit the auton state.  Be sure the lift is not elevated prior to exiting auton control, other wise it will
       * drop!!!! */
      for (L_RobotMotor = E_RobotMotorLeftWheel;
          L_RobotMotor < E_RobotMotorSz;
          L_RobotMotor = T_RobotMotor(int(L_RobotMotor) + 1))
        {
        V_RobotMotorCmndPct[L_RobotMotor] = 0.0;
        }

      /* Ok, let's indicate that we have completed the auton control. */
      V_RobatState = E_AutonComplete;
      }

    /* Need to clean this up at some point, but we need to maintain control of the lift mechanism */
    V_RobotMotorCmndPct[E_RobotMotorLift]  = Control_PID( V_IntakeLiftHeightDesired,
                                                          V_IntakePosition,
                                                         &V_IntakePositionErrorPrev,
                                                         &V_IntakePositionErrorIntegral,
                                                         K_Intake_PID_Gain[E_PID_Proportional],
                                                         K_Intake_PID_Gain[E_PID_Integral],
                                                         K_Intake_PID_Gain[E_PID_Derivative],
                                                         K_Intake_PID_Limit[E_PID_Proportional],
                                                         -K_Intake_PID_Limit[E_PID_Proportional],
                                                         K_Intake_PID_Limit[E_PID_Integral],
                                                         -K_Intake_PID_Limit[E_PID_Integral],
                                                         K_Intake_PID_Limit[E_PID_Derivative],
                                                         -K_Intake_PID_Limit[E_PID_Derivative],
                                                          K_IntakeCmndLimit,
                                                         -K_IntakeCmndLimit);

    V_RobotMotorCmndPct[E_RobotMotorLift] = LiftCmdDisable(V_IntakePosition,
                                                           V_IntakeLiftHeightDesired,
                                                           K_IntakeMinCmndHeight,
                                                           V_RobotMotorCmndPct[E_RobotMotorLift]);

    SmartDashboard::PutNumber("AutonMode", (double)L_AutonOption);
    SmartDashboard::PutNumber("L_AutonStep", (double)L_AutonStep);

    UpdateLED_Output(V_RobatState,
                     false,
                     V_RobotMotorCmndPct[E_RobotMotorWinch],
                     L_TurnSignal,
                     V_LED_State0,
                     V_LED_State1,
                     V_LED_State2,
                     V_LED_State3);

    UpdateActuatorCmnds(_talon0,
                        _talon1,
                        _talon2,
                        _talon3,
                        _talon4,
                        _talon5,
                        Spark1,
                        Spark2,
                        Talon_PWM0,
                        Talon_PWM1,
                        Talon_PWM2,
                        V_RobotMotorCmndPct);

    UpdateSmartDashboad();

    Wait(C_ExeTime);
    }
  }

/******************************************************************************
 * Function:     TestPeriodic
 *
 * Description:  This is the function that is called during the periodic period
 *               for the test period.
 ******************************************************************************/
  void TestPeriodic()
  {
    bool LED_State0 = Prefs->GetDouble("LED_State0", 0);
    bool LED_State1 = Prefs->GetDouble("LED_State1", 0);
    bool LED_State2 = Prefs->GetDouble("LED_State2", 0);
    bool LED_State3 = Prefs->GetDouble("LED_State3", 0);

      while (IsOperatorControl() && IsEnabled())
         {
           V_LED_State0->Set(LED_State0);
           V_LED_State1->Set(LED_State1);
           V_LED_State2->Set(LED_State2);
           V_LED_State3->Set(LED_State3);

           Wait(C_ExeTime);
         }
  }

};

START_ROBOT_CLASS(Robot)
