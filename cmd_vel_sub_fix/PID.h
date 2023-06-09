#ifndef __PID_H
#define __PID_H

typedef struct PID_C
{ 
  long SumError;      
  float Proportion;  
  float Integral;   
  float Derivative;  
  float LastError;      //Error[-1] 
  float PrevError;      //Error[-2] 
} PID_C;


class INCPID {
public:
  INCPID();
  void Init();
  float CalcInc(float SetPoint, float CurrentPoint);
private:
//  PID_C *sptr;
 };

#endif
