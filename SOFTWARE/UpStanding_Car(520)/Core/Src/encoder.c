#include "encoder.h"

int Read_Speed(TIM_HandleTypeDef *htim)
{
	int temp;
	//存地址htim的计数器值
	temp=(short)__HAL_TIM_GetCounter(htim);
	//置零
	__HAL_TIM_SetCounter(htim,0);
	//返回计数值
	return temp;
}
