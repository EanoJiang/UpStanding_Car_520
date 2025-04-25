#include "motor.h"

#define PWM_MAX 7200
#define PWM_MIN -7200
extern TIM_HandleTypeDef htim1;

//绝对值
int abs(int p)
{
	if(p>0)
		return p;
	else
		return -p;
}

//设置电机速度
void Load(int moto1,int moto2)			//-7200~7200
{
	//正转
	if(moto1>0)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	}
	//反转
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
	}
	//控制电机 1 的转速
	//原理：设置定时器 TIM1 通道 4 的比较寄存器（CCR4）值，改变该通道的 PWM 占空比
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,abs(moto1));
	if(moto2>0)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,abs(moto2));
}

//限幅函数
void Limit(int *motoA,int *motoB)
{
	if(*motoA>PWM_MAX)*motoA=PWM_MAX;
	if(*motoA<PWM_MIN)*motoA=PWM_MIN;
	if(*motoB>PWM_MAX)*motoB=PWM_MAX;
	if(*motoB<PWM_MIN)*motoB=PWM_MIN;
}