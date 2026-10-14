/***********************************************
公司：轮趣科技(东莞)有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-09-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2022-09-05

All rights reserved
***********************************************/
#include "show.h"
/**************************************************************************
Function: OLED display
Input   : none
Output  : none
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/
void oled_show(void)
{
	int Voltage_Show;
	Voltage_Show=Voltage*100; 
	memset(OLED_GRAM,0, 128*8*sizeof(u8));	//GRAM清零但不立即刷新，防止花屏
	//第一行
	OLED_ShowString(0,0,"IDR:");
	if(Board_Ver==1){
	if(DH1)  OLED_ShowString(50,0,"1");
	 else     OLED_ShowString(50,0,"0");
	 
	 if(DH2)  OLED_ShowString(65,0,"1");
	 else     OLED_ShowString(65,0,"0");
	 
	 if(DH3)  OLED_ShowString(80,0,"1");
	 else     OLED_ShowString(80,0,"0");
	 
	 if(DH4)  OLED_ShowString(95,0,"1");
	 else     OLED_ShowString(95,0,"0");
	}
	else if(Board_Ver==2)
	{
	if(DH1_V2)  OLED_ShowString(50,0,"1");
	 else     OLED_ShowString(50,0,"0");
	 
	 if(DH2_V2)  OLED_ShowString(65,0,"1");
	 else     OLED_ShowString(65,0,"0");
	 
	 if(DH3_V2)  OLED_ShowString(80,0,"1");
	 else     OLED_ShowString(80,0,"0");
	 
	 if(DH4_V2)  OLED_ShowString(95,0,"1");
	 else     OLED_ShowString(95,0,"0");
	}
	//第二行
	  OLED_ShowString(0,10,"Vx");
	OLED_ShowNumber(20,10,base_speed_mm,4,12);
	OLED_ShowString(60,10,"Vz");
	if(turn_diff<0) OLED_ShowString(80,10,"-");
	else OLED_ShowString(80,10,"+");
	OLED_ShowNumber(90,10,fabs(turn_diff),4,12);
	//第三行
	OLED_ShowNumber(0,20,Turn90Angle,3,12);
	OLED_ShowNumber(40,20,TurnMaxAngle,3,12);
	OLED_ShowNumber(80,20,TurnMidAngle,3,12);
	OLED_ShowNumber(100,20,TurnMinAngle,3,12);
	//第四行
	 OLED_ShowString(0,30,"L:");
	 if( MOTOR_A.Target<0)	OLED_ShowString(15,30,"-"),
													OLED_ShowNumber(20,30,-MOTOR_A.Target*1000,5,12);
	 else                 	OLED_ShowString(15,30,"+"),
													OLED_ShowNumber(20,30, MOTOR_A.Target*1000,5,12);  
	 if( MOTOR_A.Encoder<0)	OLED_ShowString(60,30,"-"),
													OLED_ShowNumber(75,30,-MOTOR_A.Encoder*1000,5,12);
	 else                 	OLED_ShowString(60,30,"+"),
													OLED_ShowNumber(75,30, MOTOR_A.Encoder*1000,5,12);
	 //第五行
	 OLED_ShowString(0,40,"R:");
	 if( MOTOR_B.Target<0)	OLED_ShowString(15,40,"-"),
													OLED_ShowNumber(20,40,-MOTOR_B.Target*1000,5,12);
	 else                 	OLED_ShowString(15,40,"+"),
													OLED_ShowNumber(20,40,  MOTOR_B.Target*1000,5,12);  
		
	 if( MOTOR_B.Encoder<0)	OLED_ShowString(60,40,"-"),
													OLED_ShowNumber(75,40,-MOTOR_B.Encoder*1000,5,12);
	 else                 	OLED_ShowString(60,40,"+"),
													OLED_ShowNumber(75,40, MOTOR_B.Encoder*1000,5,12);
	//第六行
	OLED_ShowString(0,50,"Vol:");  
	OLED_ShowNumber(40,50,Voltage_Show/100,2,12);
	OLED_ShowString(56,50,".");
	OLED_ShowNumber(64,50,Voltage_Show%100,2,12);
	OLED_ShowString(80,50,"V");
	if(EN==1)   OLED_ShowString(100,50,"O N");  
	else              OLED_ShowString(100,50,"OFF"); 
	
	OLED_Refresh_Gram();	
}

float base_vol=11.5f;
#define VOL_COUNT 100
float VolMean_Filter(float data)
{
    u8 i;
    double Sum_Speed = 0;
    float Filter_Speed;
    static  float Speed_Buf[VOL_COUNT]= {0};
	
	/*----------- 数组初始化 -----------*/
	static u8 once=1;
	if(once)
	{
		once=0;
		for(i=0;i<VOL_COUNT;i++)
			Speed_Buf[i]=base_vol;
	}
	/*-------------------------------*/
	
    for(i = 1 ; i<VOL_COUNT; i++)
    {
        Speed_Buf[i - 1] = Speed_Buf[i];
    }
    Speed_Buf[VOL_COUNT - 1] =data;

    for(i = 0 ; i < VOL_COUNT; i++)
    {
        Sum_Speed += Speed_Buf[i];
    }
    Filter_Speed = (float)(Sum_Speed / VOL_COUNT);
    return Filter_Speed;
}

//Sends the parameter's flag bit to the Bluetooth APP
//向蓝牙APP发送参数的标志位
u8 PID_Send;
extern int temp;
void APP_Show(void)
{    
	 static u8 flag_show;
	 int Left_Figure,Right_Figure,Voltage_Show;
	
	 //The battery voltage is processed as a percentage
	 //对电池电压处理成百分比形式
	 Voltage_Show=(Voltage*1000-10000)/27;
	 if(Voltage_Show>100)Voltage_Show=100; 
	
	 //Wheel speed unit is converted to 0.01m/s for easy display in APP
	 //车轮速度单位转换为0.01m/s，方便在APP显示
	 Left_Figure=MOTOR_A.Encoder*100;  if(Left_Figure<0)Left_Figure=-Left_Figure;	
	 Right_Figure=MOTOR_B.Encoder*100; if(Right_Figure<0)Right_Figure=-Right_Figure;
	
	 //Used to alternately print APP data and display waveform
	 //用于交替打印APP数据和显示波形
	 flag_show=!flag_show;
	
	 if(PID_Send==1) 
	 {
		 //Send parameters to the APP, the APP is displayed in the debug screen
		 //发送参数到APP，APP在调试界面显示
		 printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d:}$",(int)Velocity_KP,(int)Velocity_KI,(int)BaseSpeed,(int)(Turn90Angle) ,(int) (TurnMaxAngle),(int)(TurnMidAngle),(int)(TurnMinAngle),(int)ForwardLimit,(int)0);   
		 PID_Send=0;	                                                                 
	 }	                                                                                  
	 else	if(flag_show==0)                                                              
	 {
		 //Send parameters to the APP and the APP will be displayed on the front page
		 //发送参数到APP，APP在首页显示
		 printf("{A%d:%d:%d:%d}$",(u8)Left_Figure,(u8)Right_Figure,Voltage_Show,(int)imu.gyro.z);
	 }
	 else
	 {
		 //Send parameters to the APP, the APP is displayed in the waveform interface
		 //发送参数到APP，APP在波形界面显示
		 printf("{B%d:%d:%d}$",(int)imu.gyro.x,(int)imu.gyro.y,(int)imu.gyro.z);
	 }
}



