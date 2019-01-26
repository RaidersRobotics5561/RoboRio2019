/*
  SignalFilter.cpp

   Created on: Feb 14, 2018
       Author: 5561
 */



/******************************************************************************
 * Function:     LagFilter
 *
 * Description:  Simple first order lag filter.
 ******************************************************************************/
double LagFilter(double L_FilterGain,
                 double L_SpeedRaw,
                 double L_SpeedFiltPrev)
  {
  return (L_FilterGain * L_SpeedRaw + (1 - L_FilterGain) * L_SpeedFiltPrev);
  }


/******************************************************************************
 * Function:     DeadBand
 *
 * Description:  Simple dead band.
 ******************************************************************************/
double DeadBand(double L_RawSignal,
                double L_SignalLowerBand,
                double L_SignalHighBand)
  {
  double L_FilteredSignal = 0.0;

  if ((L_RawSignal >= L_SignalHighBand) ||
      (L_RawSignal <= L_SignalLowerBand))
    {
    L_FilteredSignal = L_RawSignal;
    }

  return (L_FilteredSignal);
  }
