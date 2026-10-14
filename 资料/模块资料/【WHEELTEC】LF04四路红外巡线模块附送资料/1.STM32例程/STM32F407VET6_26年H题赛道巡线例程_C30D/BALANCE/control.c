#include "control.h"
//机器人软件失能标志位
u8 Flag_Stop=0;
//速度控制PID参数
float Velocity_KP=500,Velocity_KI=500;
Encoder OriginalEncoder;//编码器原始数据   
//电机的参数结构体
Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;

/**
 * @brief  TIM7中断服务函数--2ms一次
 * @param  无
 * @retval 无
 */
void TIM7_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
    {
        /* 清除中断标志位 */
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		Get_Velocity_Form_Encoder();
        if(Board_Ver==1)
		{
			IRDM_line_inspection();	//红外巡线数据读取_C30D_V1.0
		}
		else if(Board_Ver==2)
		{
			IRDM_line_inspection_V2();	//红外巡线数据读取_C30D_V2.0
		}
		if(Turn_Off(Voltage)==0||EN) 
		{
			//Speed closed-loop control to calculate the PWM value of each motor, 
			//PWM represents the actual wheel speed					 
			//速度闭环控制计算各电机PWM值，PWM代表车轮实际转速
			MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
			MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
			MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
			MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
			Limit_Pwm(16700);
			//驱动电机
			Set_Pwm( MOTOR_A.Motor_Pwm,MOTOR_B.Motor_Pwm, 0, 0, 0    );
		}else Set_Pwm(0,0,0,0,0);
		//LED灯每500次（1s）闪烁一次
		LED_Flash_Ctrl(500);
    }
}
/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	//Retrieves the original data of the encoder
	//获取编码器的原始数据
	float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
	OriginalEncoder.A=Read_Encoder(2);	
	OriginalEncoder.B=Read_Encoder(3);	
	OriginalEncoder.C=Read_Encoder(4);	
	OriginalEncoder.D=Read_Encoder(5);	
	
	Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D;

	//The encoder converts the raw data to wheel speed in m/s
	//编码器原始数据转换为车轮速度，单位m/s
	MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Perimeter/(EncoderMultiples*ENCODER_RESOLUTION*MOTOR_GEAR_RATIO);  
	MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Perimeter/(EncoderMultiples*ENCODER_RESOLUTION*MOTOR_GEAR_RATIO);  
	MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Perimeter/(EncoderMultiples*ENCODER_RESOLUTION*MOTOR_GEAR_RATIO); 
	MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Perimeter/(EncoderMultiples*ENCODER_RESOLUTION*MOTOR_GEAR_RATIO); 
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	static float Bias,Pwm,Last_bias;
	Bias=Target-Encoder; //Calculate the deviation //计算偏差
	Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	if(Pwm>16700)Pwm=16700;
	if(Pwm<-16700)Pwm=-16700;
	if(Turn_Off(Voltage)==1||!EN) Pwm=0;
	Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
	static float Bias,Pwm,Last_bias;
	Bias=Target-Encoder; //Calculate the deviation //计算偏差
	Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	if(Pwm>16700)Pwm=16700;
	if(Pwm<-16700)Pwm=-16700;
	if(Turn_Off(Voltage)==1||!EN) Pwm=0;
	Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 if(Turn_Off(Voltage)==1||!EN) Pwm=0;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 if(Turn_Off(Voltage)==1||!EN) Pwm=0;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}

float max_return(float intput1,float intput2)
{
	if(intput1>intput2)
return intput1;
	else
return intput2;	


}

/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
**************************************************************************/
u8 Turn_Off( float voltage)
{
	    u8 temp;
		if(voltage<7||EN==0)
		{	   
			temp=1;      
			PWMA1=0;PWMA2=0;
			PWMB1=0;PWMB2=0;		
			PWMC1=0;PWMC1=0;	
			PWMD1=0;PWMD2=0;					
		}
		else
		temp=0;
		return temp;			
}

/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_a<0)			PWMA1=16799,PWMA2=16799+motor_a-200;//200为电机死区
	else 	            PWMA2=16799,PWMA1=16799-motor_a-200;
	
	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_b<0)			PWMB1=16799,PWMB2=16799+motor_b-200;
	else 	            PWMB2=16799,PWMB1=16799-motor_b-200;
//  PWMB1=10000,PWMB2=5000;

	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_c<0)			PWMC1=16799,PWMC2=16799+motor_c-200;
	else 	            PWMC2=16799,PWMC1=16799-motor_c-200;
	
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_d<0)			PWMD1=16799,PWMD2=16799+motor_d-200;
	else 	            PWMD2=16799,PWMD1=16799-motor_d-200;
	
	//Servo control
	//舵机控制
	Servo_PWM =servo;
}



