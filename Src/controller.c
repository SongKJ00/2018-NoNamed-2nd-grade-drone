#include "controller.h"


#define GAIN ((max_duty - min_duty) / (max_pulse - min_pulse))

#define min_duty 4619.0
#define max_duty 8189.0

#define min_pulse 352.0
#define max_pulse 1696.0

float pid_output[4];

void pid_control()
{
  int i;
  float dt = 0.001;
  float kp[3], ki[3], kd[3];

  static float iTerm[3] = { 0 };

  float setpoint[3];
  float error[3];
  float pTerm[3];
  float dTerm[3];

  kp[ROLL]  = 6.0;
  kp[PITCH] = 6.0;
  kp[YAW]   = 6.0;

  ki[ROLL]  = 0.0;
  ki[PITCH] = 0.0;
  ki[YAW]   = 0.0;	

  kd[ROLL]  = 2.0;
  kd[PITCH] = 2.0;
  kd[YAW]   = 2.0;

  setpoint[ROLL]  = (sbus_ch_data[CH4] - 352) / ((float)(1344) / 60) - 30;
  setpoint[PITCH] = (sbus_ch_data[CH2] - 352) / ((float)(1344) / 60) - 30;
  setpoint[YAW]   = (sbus_ch_data[CH1] - 352) / ((float)(1344) / 300) - 150;

  for(i = 0; i < 3; i++)
  {
    /* New Code Begin */
    /* When target axes are ROLL or PITCH */
    if(i == ROLL || i == PITCH)
    {
      error[i]  = setpoint[i]  - mti3.euler[i];
    }
    
    /* when target axis is YAW */
    else
    {
      error[i] = setpoint[i] - mti3.pqr[YAW];
    }
    /* New Code End */  
          
    pTerm[i] =  kp[i] * error[i];
    iTerm[i] += error[i] * dt;
    dTerm[i] = -kd[i] * mti3.pqr[i];

    pid_output[i] = pTerm[i] + (ki[i] * iTerm[i]) + dTerm[i];
  }

  pid_output[3] = sbus_ch_data[CH3];

  /* for(i = 0; i < 4; i++)
    printf("%f\n\r", pid_output[i]);

  printf("\n\r");*/
}

void controller(){
    pid_control();
	
    /* Main 함수에서 한 번만 호출하도록 */
    //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
 
    int16_t value_output[4];
    
    int k;
    
    if( pid_output[3] <= 352)
    {
      for(k=0; k<4; k++)
      {
          value_output[k] = 352;
      }
    }
    
    else 
    {      
      value_output[0] = (int16_t)(-pid_output[ROLL] + pid_output[PITCH] + pid_output[YAW] + pid_output[3]); 
      value_output[1] = (int16_t) (pid_output[ROLL] + pid_output[PITCH] - pid_output[YAW] + pid_output[3]); 
      value_output[2] = (int16_t) (pid_output[ROLL] - pid_output[PITCH] + pid_output[YAW] + pid_output[3]); 
      value_output[3] = (int16_t)(-pid_output[ROLL] - pid_output[PITCH] - pid_output[YAW] + pid_output[3]); 
    }
    
    
    for(k=0; k<4; k++)
    {
      if(value_output[k]<=352)
      {
        value_output[k] = 352;
      }
      else if(value_output[k] >= 1696)
      {
        value_output[k] = 1696;
      }
    }
    
    htim1.Instance -> CCR1 = (int16_t)(value_output[0] * GAIN) + 3684;
    htim1.Instance -> CCR2 = (int16_t)(value_output[1] * GAIN) + 3684;
    htim1.Instance -> CCR3 = (int16_t)(value_output[2] * GAIN) + 3684;
    htim1.Instance -> CCR4 = (int16_t)(value_output[3] * GAIN) + 3684;
}

