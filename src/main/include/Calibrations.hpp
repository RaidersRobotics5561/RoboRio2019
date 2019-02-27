#include "Robot.h"
#include <frc/Driverstation.h>

/* K_DesiredDriveSpeed: Look up table used for telop driving.  This is 
 referenced relative to the joystick axis and also uses 
 K_DesiredDriveSpeedAxis as the axis lookup. (RPM) */
const double K_DesiredDriveSpeed[20] = {-300,  //-0.95
                                        -223.125,  //-0.85
                                         -168.75, //-0.75
                                         -112.5,//-0.65
                                         -75,  //-0.55
                                         -56.25, //-0.45
                                         -37.5,  //-0.35
                                          -18.75,  //-0.25
                                          -5,  //-0.15
                                           0.00,  //-0.10
                                           0.00,  // 0.10
                                           5,  // 0.15
                                           18.75,  // 0.25
                                          37.5,  // 0.35
                                          56.25,  // 0.45
                                          75,  // 0.55
                                          112.5,  // 0.65
                                          168.75,  // 0.75
                                         223.125,  // 0.85
                                         300.00}; // 0.95

/* K_DesiredDriveSpeedAxis: Look up table axis for desired drive speed 
 K_DesiredDriveSpeed. (PCT) */
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

/* K_RotateGain: Rotation gain for arcade drive. */
const double K_RotateGain = 1;

/* K_RobotShimmySpeed: This is the desired wheel speed for one side of the robot during shimmy. (RPM)*/
const double K_RobotShimmySpeed = 120;

/* K_RobotShimmyDistance: This is the amount of measured distance that the robot will rotate during one of the shimmys. (???)*/
const double K_RobotShimmyDistance = 1200;

/* K_EndMatchWarningTime: This is the amount of time till the end of the game at which the LED lights will begin to
                          indicate to the driver that the game is about to end.  Typically this is 30 seconds. (seconds)*/
const double K_EndMatchWarningTime  =  30.0;

/* K_SandStormTime: This is the amount of time in sand storm. (seconds)*/
const double K_SandStormTime  =  120.0;