#include "sr04.h"
#include "pid.h"
uint16_t count;
float distance;
extern TIM_HandleTypeDef htim3;

//时延函数
void RCCdelay_us(uint32_t udelay)
{
  __IO uint32_t Delay = udelay * 72 / 8;//(SystemCoreClock / 8U / 1000000U)
    //¼ûstm32f1xx_hal_rcc.c -- static void RCC_Delay(uint32_t mdelay)
  do
  {
    __NOP();
  }
  while (Delay --);
}

//手动发送一个时延信号
void GET_Distance(void)
{

	//上升沿
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
	//延时12us
	RCCdelay_us(12);
	//下降沿
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
}

//外部中断回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_2)
	{
		//上升沿
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)==GPIO_PIN_SET)
		{
			//清零
			__HAL_TIM_SetCounter(&htim3,0);
			HAL_TIM_Base_Start(&htim3);
		}
		//下降沿
		else
		{
			//停止计数
			HAL_TIM_Base_Stop(&htim3);
			//获取计数值
			count=__HAL_TIM_GetCounter(&htim3);
			//计算距离
			//c=340 m/s（即 34000 cm/s）
			//来回单程距离对应的每微秒位移 = 0.034 cm/μs ÷ 2 = 0.017 cm/μs
			distance=count*0.017;
		}
	}
	//如果触发中断的是MPU6050 DMP 中断，就直接控制
	if(GPIO_Pin==GPIO_PIN_5)
		Control();
}
