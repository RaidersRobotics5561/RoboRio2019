/*
  LookUp.cpp

   Created on: Feb 11, 2018
       Author: 5561

       This file contains the functions necessary for lookup tables.
 */
#include "const.h"


double LookUp1D_Table(const double *L_X_Axis,
                      const double *L_TableData1D,
                            int     L_AxisSize,
                            int     L_CalArraySize,
                            double  L_Input)
  {
  int    L_Index        = 0;
  double L_LookupX1     = 0.0;
  double L_LookupX2     = 0.0;
  double L_LookupX_Diff = 0.0;
  double L_LookupY1     = 0.0;
  double L_LookupY2     = 0.0;
  double L_LookupY_Diff = 0.0;
  double L_LookupDiv    = 0.0;
  bool L_LookupPt1Found = false;
  double L_Output       = 0.0;

  /* Table length MUST equal axis length. */
  if (L_CalArraySize == L_AxisSize)
    {
    if (L_Input >= (L_X_Axis[L_AxisSize - 1]))
      {
      // We have gone off or are at the end of the axis
      return (L_TableData1D[L_AxisSize - 1]);
      }
    else if (L_Input <= (L_X_Axis[0]))
      {
      // We have gone off or are at the beginning of the axis
      return (L_TableData1D[0]);
      }
    else
      {
      for (L_Index = 0; ((L_Index < (L_AxisSize - 1)) && (L_LookupPt1Found == false)) ; L_Index++)
        {
        if ((L_Input >= L_X_Axis[L_Index])     &&
            (L_Input <  L_X_Axis[L_Index + 1]) &&
            (L_LookupPt1Found == false))
          {
          L_LookupX1 = L_X_Axis[L_Index];
          L_LookupY1 = L_TableData1D[L_Index];
          L_LookupX2 = L_X_Axis[L_Index + 1];
          L_LookupY2 = L_TableData1D[L_Index + 1];
          L_LookupPt1Found = true;

          L_Index = L_AxisSize;
          }
        }

      if ((L_LookupPt1Found == true))
        {
        L_LookupX_Diff = L_LookupX2 - L_LookupX1;
        L_LookupY_Diff = L_LookupY2 - L_LookupY1;
        if (L_LookupX_Diff != 0.0)
          {
          /* Protect for zero division */
          L_LookupDiv = L_LookupY_Diff / L_LookupX_Diff;
          }
        else
          {
          L_LookupDiv = 0.0;
          }
        L_Output = L_LookupY1 + (L_Input-L_LookupX1) * L_LookupDiv;

        return L_Output;
        }
      }
    }

  // Not in range...
  return 0;
  }
