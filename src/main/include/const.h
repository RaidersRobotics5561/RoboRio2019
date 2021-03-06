
#include <iostream>
#include <string>

#include <frc/IterativeRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <frc/Ultrasonic.h>
#include <frc/Joystick.h>
#include <frc/Spark.h>

using namespace frc;

const double C_ExeTime = 0.01;

const int K_TimeoutMs = 10;
const double K_WheelPulseToRev = 4100;

// K_RobotType: For practice bot, set to 1.  For Comp, set to -1.
const double K_RobotType = -1.0;