
#include "Calibrations.hpp"
#include "LookUp.hpp"

double K_LukeStopperRamp = 60;
double K_LukeStopperRampLow = 40;
double K_LukeStopperThreashold = 75;

double K_LukeStopperMaxDelta = 60;

double Control_PID(double  L_DesiredSpeed,
                   double  L_CurrentSpeed,
                   double *L_ErrorPrev,
                   double *L_IntegralPrev,
                   double  L_ProportionalGx,
                   double  L_IntegralGx,
                   double  L_DerivativeGx,
                   double  L_ProportionalUpperLimit,
                   double  L_ProportionalLowerLimit,
                   double  L_IntegralUpperLimit,
                   double  L_IntegralLowerLimit,
                   double  L_DerivativeUpperLimit,
                   double  L_DerivativeLowerLimit,
                   double  L_OutputUpperLimit,
                   double  L_OutputLowerLimit)
  {
  double L_Error        = 0.0;
  double L_Proportional = 0.0;
  double L_Integral     = 0.0;
  double L_Derivative   = 0.0;
  double L_OutputCmnd   = 0.0;

  L_Error = L_DesiredSpeed - L_CurrentSpeed;

  L_Proportional = L_Error * L_ProportionalGx;

  L_Integral = *L_IntegralPrev + (L_Error * L_IntegralGx);

  L_Derivative = L_DerivativeGx * (*L_ErrorPrev / 0.01);

  *L_ErrorPrev = L_Error;

  if (L_Proportional > L_ProportionalUpperLimit)
    {
    L_Proportional = L_ProportionalUpperLimit;
    }
  else if (L_Proportional < L_ProportionalLowerLimit)
    {
    L_Proportional = L_ProportionalLowerLimit;
    }

  if (L_Integral > L_IntegralUpperLimit)
    {
    L_Integral = L_IntegralUpperLimit;
    }
  else if (L_Integral < L_IntegralLowerLimit)
    {
    L_Integral = L_IntegralLowerLimit;
    }

  if (L_Derivative > L_DerivativeUpperLimit)
    {
    L_Derivative = L_DerivativeUpperLimit;
    }
  else if (L_Derivative < L_DerivativeLowerLimit)
    {
    L_Derivative = L_DerivativeLowerLimit;
    }

  /* Ok, lets record the integral to use next loop: */
  *L_IntegralPrev = L_Integral;

  /* Lets add all three controllers. */
  L_OutputCmnd = L_Proportional + L_Integral + L_Derivative;

  /* This is kind of redundant, but lets limit the output to the min and max
   * allowed for the controller: */
  if (L_OutputCmnd > L_OutputUpperLimit)
    {
    L_OutputCmnd = L_OutputUpperLimit;
    }
  else if (L_OutputCmnd < L_OutputLowerLimit)
    {
    L_OutputCmnd = L_OutputLowerLimit;
    }

  return L_OutputCmnd;
}

double LukeStoppers(double L_DesiredSpeed,
                    double L_CurrentSpeed,
                    double L_RampRate,
                    double L_RampRateLow,
                    double L_Threshold,
                    double L_MaxDelta)
  {
  double L_FinalDesiredSpeed = L_DesiredSpeed;

  if ((fabs(L_DesiredSpeed) < 1.0) &&
      (fabs(L_CurrentSpeed) > 1.0))
    {
      if (L_CurrentSpeed > 0.0)
      {
        if(L_CurrentSpeed >= L_Threshold)
        {
          L_FinalDesiredSpeed = L_CurrentSpeed - L_RampRate;
        } else
        {
          L_FinalDesiredSpeed = L_CurrentSpeed - L_RampRateLow;
        }

        if (L_FinalDesiredSpeed < 0.0)
        {
          L_FinalDesiredSpeed = 0.0;
        }
      }
    else
      {
        if(L_CurrentSpeed <= -L_Threshold)
        {
          L_FinalDesiredSpeed = L_CurrentSpeed + L_RampRate;
        } else
        {
           L_FinalDesiredSpeed = L_CurrentSpeed + L_RampRateLow;
        }

        if (L_FinalDesiredSpeed > 0.0)
        {
          L_FinalDesiredSpeed = 0.0;
        }
      }
    }

  if(fabs(L_CurrentSpeed - L_FinalDesiredSpeed) >= L_MaxDelta ) {
    if(L_FinalDesiredSpeed - L_CurrentSpeed > 0) {
      L_FinalDesiredSpeed = L_CurrentSpeed + L_MaxDelta;
    }
    else {
      L_FinalDesiredSpeed = L_CurrentSpeed - L_MaxDelta;
    }
  }

  return (L_FinalDesiredSpeed);
  }


double DesiredSpeed(double L_JoystickAxis,
                    double L_CurrentSpeed)
  {
  double L_DesiredDriveSpeed = 0.0;
  int L_AxisSize = (int)(sizeof(K_DesiredDriveSpeedAxis) / sizeof(K_DesiredDriveSpeed[0]));
  int L_CalArraySize = (int)(sizeof(K_DesiredDriveSpeed) / sizeof(K_DesiredDriveSpeed[0]));

  L_DesiredDriveSpeed = LookUp1D_Table(&K_DesiredDriveSpeedAxis[0],
                                       &K_DesiredDriveSpeed[0],
                                       L_AxisSize,
                                       L_CalArraySize,
                                       L_JoystickAxis);

  L_DesiredDriveSpeed = LukeStoppers(L_DesiredDriveSpeed,
                                     L_CurrentSpeed,
                                     K_LukeStopperRamp,
                                     K_LukeStopperRampLow,
                                     K_LukeStopperThreashold,
                                     K_LukeStopperMaxDelta);

  return L_DesiredDriveSpeed;
  }
