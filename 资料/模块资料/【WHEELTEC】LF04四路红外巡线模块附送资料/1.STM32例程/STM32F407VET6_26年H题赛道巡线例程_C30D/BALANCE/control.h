#ifndef __CONTROL_H
#define __CONTROL_H	 
#include "sys.h"

#define EncoderMultiples  4				//编码器倍频数，取决于编码器初始化设置
#define CONTROL_FREQUENCY 500			//编码器数据读取频率
#define	Black_WheelDiameter   0.065f	//轮胎直径
#define Perimeter	0.204203519f		//轮子周长(单位:m)
#define MOTOR_GEAR_RATIO       28.0f    // 电机减速比
#define ENCODER_RESOLUTION     13.0f    // 编码器线数

//Motor speed control related parameters of the structure
//电机速度控制相关参数结构体
typedef struct  
{
	float Encoder;     //Read the real time speed of the motor by encoder //编码器数值，读取电机实时速度
	float Motor_Pwm;   //Motor PWM value, control the real-time speed of the motor //电机PWM数值，控制电机实时速度
	float Target;      //Control the target speed of the motor //电机目标速度值，控制电机目标速度
	float Velocity_KP; //Speed control PID parameters //速度控制PID参数
	float	Velocity_KI; //Speed control PID parameters //速度控制PID参数
}Motor_parameter;

// Encoder structure
//编码器结构体
typedef struct  
{
	int A;      
	int B; 
	int C; 
	int D; 
}Encoder;
u8 Turn_Off( float voltage);
void Limit_Pwm(int amplitude);
void Get_Velocity_Form_Encoder(void);
int Incremental_PI_A (float Encoder,float Target);
int Incremental_PI_B (float Encoder,float Target);
int Incremental_PI_C (float Encoder,float Target);
int Incremental_PI_D (float Encoder,float Target);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
float max_return(float intput1,float intput2);
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo);

extern Encoder OriginalEncoder; //Encoder raw data //编码器原始数据   
extern Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;
extern u8 Flag_Stop;
extern float Velocity_KP,Velocity_KI;
#endif

























