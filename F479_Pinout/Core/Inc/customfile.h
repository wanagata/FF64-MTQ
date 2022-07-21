#ifndef __CUSTOMFILE_H
#define __CUSTOMFILE_H

void intialcs(TIM_HandleTypeDef *timer){
  HAL_TIM_PWM_Start(timer, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(timer, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(timer, TIM_CHANNEL_3);
}


#endif 
