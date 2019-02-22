


bool AutonLiftToHight(TalonSRX *_talon6, TalonSRX *_talon5);
bool AutonDriveLiftWheel(Spark *_spark1, Ultrasonic *_Ultrasonic);
bool AutonMainDrive(TalonSRX *_talon1, TalonSRX *_talon2, TalonSRX *_talon3, TalonSRX *_talon4, Ultrasonic *_Ultrasonic);
bool AutonRaiseBackLift(TalonSRX *_talon6);
bool AutonRaiseForwardLift(TalonSRX *_talon5);
bool MaintainBackLift(TalonSRX *_talon6);
bool MaintainForwardLift(TalonSRX *_talon5);