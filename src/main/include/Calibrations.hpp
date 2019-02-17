#include "Robot.h"

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

const double K_DesiredDriveSpeedLifted[20] = {-37.5,  //-0.95
                                        -27.9,  //-0.85
                                         -21, //-0.75
                                         -14,//-0.65
                                         -9,  //-0.55
                                         -7, //-0.45
                                         -4.7,  //-0.35
                                          -2.3,  //-0.25
                                          -0.6,  //-0.15
                                           0.00,  //-0.10
                                           0.00,  // 0.10
                                           0.6,  // 0.15
                                           2.3,  // 0.25
                                          4.7,  // 0.35
                                          7,  // 0.45
                                          9,  // 0.55
                                          14,  // 0.65
                                          21,  // 0.75
                                         27.9,  // 0.85B
                                         37.5}; // 0.95

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

/* K_FwdLimit: Forward limit for tank drive. */
const double K_FwdLimit = 1.0;

/* K_RevLimit: Reverse limit for tank drive. */
const double K_RevLimit = -0.75;

/* K_RotateGain: Rotation gain for arcade drive. */
const double K_RotateGain = 0.7;


/* K_RobotShimmyRight: Distance that the robot will shimmy.  For the right shimmy only. (inches) */
const double K_RobotShimmyRight[E_RobotShimmyRight_ShimmySz] =
    {-3.0,  //   E_RobotLeftBackwards
     -3.0,  //   E_RobotRightBackwards
      3.0,  //   E_RobotLeftForward
      3.0}; //   E_RobotRightForward

/* K_RobotShimmyLeft: Distance that the robot will shimmy.  For the left shimmy only. (inches) */
const double K_RobotShimmyLeft[E_RobotShimmyLeft_ShimmySz] =
    {-3.0,  //   E_RobotLeftBackwards
     -3.0,  //   E_RobotRightBackwards
      3.0,  //   E_RobotLeftForward
      3.0}; //   E_RobotRightForward
