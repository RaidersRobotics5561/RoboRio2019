
#include <ctre/Phoenix.h>
#include <frc/Ultrasonic.h>

bool AutonLiftToHight(TalonSRX *_talon6, TalonSRX *_talon5);
bool AutonDriveLiftWheel(Spark *_spark1);
bool AutonMainDrive(TalonSRX *_talon1, TalonSRX *_talon2, TalonSRX *_talon3, TalonSRX *_talon4);
bool AutonRaiseBackLift(TalonSRX *_talon6);
bool AutonRaiseForwardLift(TalonSRX *_talon5);