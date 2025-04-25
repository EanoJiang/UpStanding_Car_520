#include "pid.h"
#include "encoder.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"
#include "math.h"

//传感器数据变量
//编码器
int Encoder_Left,Encoder_Right;
//陀螺仪
// 角度计变量———— pitch：俯仰角度，roll：横滚角度，yaw：偏航角度
float pitch,roll,yaw;
//	角速度计变量———— gyrox:俯仰角速度，gyroy:横滚角速度，gyroz:偏航角速度
short gyrox,gyroy,gyroz;
// 加速度计变量————aacx,aacy,aacz
short	aacx,aacy,aacz;

//闭环控制中间变量
int Vertical_out,Velocity_out,Turn_out,Target_Speed,Target_turn,MOTO1,MOTO2;

//pid调参
float Med_Angle=2.438;//平衡时角度值偏移量（机械中值）
//参数
float Vertical_Kp=-252,Vertical_Kd=-3;			//直立环 数量级（Kp：0~1000、Kd：0~10）
//-420 -5
float Velocity_Kp=-0.4,Velocity_Ki=-0.002;		//速度环 数量级（Kp：0~1） 速度环的ki = kp/200
float Turn_Kp=10,Turn_Kd=0.1;											//转向环

//平衡状态标志
uint8_t stop;
uint8_t balance_state = 1; 	// 平衡状态标志：1表示平衡，0表示失去平衡
// 平衡角度阈值，超过此角度认为失去平衡
#define BALANCE_ANGLE_THRESHOLD 40.0f

extern TIM_HandleTypeDef htim2,htim4;
extern float distance;
extern uint8_t Fore,Back,Left,Right;
#define SPEED_Y 15.0f	//俯仰(前后)最大设定速度
#define SPEED_Z 150		//偏航(左右)最大设定速度 

// 检查平衡车是否处于平衡状态
uint8_t Is_Balance(float angle)
{
    // 检查角度是否在平衡范围内
    if(fabs(angle - Med_Angle) > BALANCE_ANGLE_THRESHOLD)
    {
        return 0; // 失去平衡
    }
    return 1; // 平衡状态
}

//直立环PD控制器
//输入：期望角度、真实角度、角速度
int Vertical(float Med,float Angle,float gyro_Y)
{
	int temp;
	temp=Vertical_Kp*(Angle-Med)+Vertical_Kd*gyro_Y;
	return temp;
}

//速度环PI控制器
//输入：期望速度、左编码器、右编码器
int Velocity(int Target,int encoder_L,int encoder_R)
{
	static int Err_LowOut_last,Encoder_S;
	static float a=0.7;
	int Err,Err_LowOut,temp;
	Velocity_Ki=Velocity_Kp/200;
	//1、计算偏差值
	Err=(encoder_L+encoder_R)-Target;
	//2、低通滤波
	Err_LowOut=(1-a)*Err+a*Err_LowOut_last;
	Err_LowOut_last=Err_LowOut;
	//3、积分
	Encoder_S+=Err_LowOut;
	//4、积分限幅(-20000~20000)
	Encoder_S=Encoder_S>20000?20000:(Encoder_S<(-20000)?(-20000):Encoder_S);
	if(stop==1)Encoder_S=0,stop=0;
	// 如果失去平衡，重置积分
	if(balance_state == 0)
	{
		Encoder_S = 0;
	}
	//5、速度环计算
	temp=Velocity_Kp*Err_LowOut+Velocity_Ki*Encoder_S;
	return temp;
}


//转向环PD控制器
//输入：角速度、角度值
int Turn(float gyro_Z,int Target_turn)
{
	int temp;
	temp=Turn_Kp*Target_turn+Turn_Kd*gyro_Z;
	return temp;
}

void Control(void)	//每隔10ms调用一次
{
	int PWM_out;
	//1、读取编码器和陀螺仪的数据
	Encoder_Left=Read_Speed(&htim2);
	Encoder_Right=-Read_Speed(&htim4);
	mpu_dmp_get_data(&pitch,&roll,&yaw);
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);
	
	// 检查平衡状态
	uint8_t current_balance_state = Is_Balance(roll);
	
	// 如果平衡状态发生变化
	if(balance_state != current_balance_state)
	{
		balance_state = current_balance_state;
		if(balance_state == 0) // 从平衡变为不平衡
		{
			// 失去平衡时将速度置零
			Target_Speed = 0;
			Target_turn = 0;
			stop = 1; // 触发速度积分清零
		}
	}
	
	// 如果处于不平衡状态，强制速度为0
	if(balance_state == 0)
	{
		Target_Speed = 0;
		Target_turn = 0;
		MOTO1 = 0;
		MOTO2 = 0;
		Load(0, 0); // 直接停止电机
		return; // 不平衡状态下不执行后续控制逻辑
	}
	
	//遥控
	if((Fore==0)&&(Back==0))Target_Speed=0;//未接受到前进后退指令-->速度清零，稳在原地
	if(Fore==1)
	{
		if(distance<50)
			Target_Speed--;
		else
			Target_Speed++;
	}
	if(Back==1){Target_Speed--;}//
	Target_Speed=Target_Speed>SPEED_Y?SPEED_Y:(Target_Speed<-SPEED_Y?(-SPEED_Y):Target_Speed);//限幅
	
	/*左右*/
	if((Left==0)&&(Right==0))Target_turn=0;
	if(Left==1)Target_turn-=30;	//左转
	if(Right==1)Target_turn+=30;	//右转
	Target_turn=Target_turn>SPEED_Z?SPEED_Z:(Target_turn<-SPEED_Z?(-SPEED_Z):Target_turn);//限幅( (20*100) * 100   )
	
	/*转向约束*/
	if((Left==0)&&(Right==0))Turn_Kd=0.6;//若无左右转向指令，则开启转向约束
	else if((Left==1)||(Right==1))Turn_Kd=0;//若左右转向指令接收到，则去掉转向约束
	
	
	//2、将数据传入PID控制器，计算输出结果，即左右电机转速值
	Velocity_out=Velocity(Target_Speed,Encoder_Left,Encoder_Right);
	Vertical_out=Vertical(Velocity_out+Med_Angle,roll,gyrox);
	Turn_out=Turn(gyroz,Target_turn);
	PWM_out=Vertical_out;
	MOTO1=PWM_out-Turn_out;
	MOTO2=PWM_out+Turn_out;
	Limit(&MOTO1,&MOTO2);
	Load(MOTO1,MOTO2);
}