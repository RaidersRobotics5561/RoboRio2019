bool AutonLiftToHight(TalonSRX *_talon6, TalonSRX *_talon5, double liftTarget);
bool AutonDriveLiftWheel(Spark *_spark1, Ultrasonic *_Ultrasonic);
bool AutonMainDrive(TalonSRX *_talon1, TalonSRX *_talon2, TalonSRX *_talon3, TalonSRX *_talon4, Ultrasonic *_Ultrasonic);
bool AutonRaiseBackLift(TalonSRX *_talon6, double   L_Pos, double L_End);
bool AutonRaiseForwardLift(TalonSRX *_talon5, double L_Pos);
bool MaintainBackLift(TalonSRX *_talon6, double L_Pos);
bool MaintainForwardLift(TalonSRX *_talon5, double L_Pos);
bool AutonMainDriveRev(TalonSRX *_talon1, TalonSRX *_talon2, TalonSRX *_talon3, TalonSRX *_talon4, Ultrasonic *_Ultrasonic, double L_Spd);
bool AutonDriveLiftWheelOpenLoop(Spark *_spark1, double L_Time);
bool AutonDropToHight(TalonSRX *_talon6, TalonSRX *_talon5, double liftTarget);
bool AutonRaiseForwardDrop(TalonSRX *_talon5,
                           double    L_Pos,
                           double    L_End);
bool AutonDropBackLift(TalonSRX *_talon6,
                        double   L_Pos,
                        double   L_End);