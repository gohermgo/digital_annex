#include "PID.h"

PID_C sPID; 
PID_C *sptr = &sPID; 
INCPID::INCPID() 
{ 
}

void INCPID::Init(void) 
{ 
  sptr->SumError = 0; 
  sptr->LastError = 0;    //Error[-1] 
  sptr->PrevError = 0;    //Error[-2] 
  sptr->Proportion = 10; 
  sptr->Integral =0.1;  
  sptr->Derivative = 0;  
}

float INCPID::CalcInc(float SetPoint, float CurrentPoint) 
{ 
  float iError = 0, iIncpid = 0;                       
  iError = SetPoint - CurrentPoint;             //Calculate the current error
  iIncpid = (sptr->Proportion) * iError        //E[k]
   - sptr->Integral * sptr->LastError    //E[k-1]
   + sptr->Derivative * sptr->PrevError; //E[k-2]

  //Store error 
  sptr->PrevError = sptr->LastError; 
  sptr->LastError = iError; 

  return iIncpid;                              //Return the increaing value 
}
