/*
  SignalFilter.hpp

   Created on: Feb 15, 2018
       Author: 5561
 */
#ifndef SRC_ROBORIO2018_SIGNALFILTER_HPP_
#define SRC_ROBORIO2018_SIGNALFILTER_HPP_


extern double LagFilter(double L_FilterGain,
                        double L_SpeedRaw,
                        double L_SpeedFiltPrev);

extern double DeadBand(double L_RawSignal,
                       double L_SignalLowerBand,
                       double L_SignalHighBand);


#endif /* SRC_ROBORIO2018_SIGNALFILTER_HPP_ */
