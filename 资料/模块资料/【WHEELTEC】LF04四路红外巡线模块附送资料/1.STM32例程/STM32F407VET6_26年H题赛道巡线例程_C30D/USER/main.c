#include "sys.h"
//imu数据结构体
IMU_DATA_t imu;
int Board_Ver = 0;
int Car_Mode_Show,POT_val,Divisor_Mode;


int main(void)
{ 
	SystemInit();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//===设置系统中断优先级分组2
    delay_init(168);
	I2C_GPIOInit();
	//如果IMU为MPU6050,则是旧版C30D
	if( MPU6050_DEFAULT_ADDRESS == MPU6050_getDeviceID() )
	{
		Board_Ver = 1;
		V1_0_LED_Init();
		TrackModule_Init();
	}
	//如果IMU型号为ICM20948,则是新版C30D
	else if( REG_VAL_WIA == ICM20948_getDeviceID() )//读取ICM20948 id
	{
		Board_Ver = 2;
		V2_0_LED_Init();
		TrackModule_Init_V2();
	}
	else //无法识别的陀螺仪,复位系统
	{
		NVIC_SystemReset();
	}
	OLED_Init();
	OLED_Clear();
	OLED_Display_On();
	uart2_init(9600);
	//ADC引脚初始化，用于读取电池电压与电位器档位，电位器档位决定小车开机后的小车适配型号
    Adc_Init();
    Adc_POWER_Init();
    //初始化与使能开关连接的硬件接口
    Enable_Pin();
	 //定时器12用作舵机的PWM接口
    TIM12_SERVO_Init(9999,84-1);  //APB1的时钟频率为84M , 频率=84M/((9999+1)*(83+1))=100Hz
    //初始化电机速度控制以及，用于控制电机速度，PWM频率10KHZ
    //APB2时钟频率为168M，满PWM为16799，频率=168M/((16799+1)*(0+1))=10k
    TIM1_PWM_Init(16799,0);
    TIM9_PWM_Init(16799,0);
    TIM10_PWM_Init(16799,0);
    TIM11_PWM_Init(16799,0);
    //编码器A初始化，用于读取电机A的实时速度
    Encoder_Init_TIM2();
    //编码器B初始化，用于读取电机B的实时速度
    Encoder_Init_TIM3();
    //编码器C初始化，用于读取电机C的实时速度
    Encoder_Init_TIM4();
    //编码器D初始化，用于读取电机D的实时速度
    Encoder_Init_TIM5();
	//初始化定时器7，2ms定时中断用于控制小车运动
	TIM7_Init();
	while(1)
	{	
		//Read the battery voltage //读取电池电压
		for(int i=0;i<100;i++)
		{
			Voltage_All+=Get_battery_volt(); 
		}
		Voltage=Voltage_All/100;
		Voltage = VolMean_Filter(Voltage);
		Voltage_All=0;
		oled_show();//OLED屏显示
		APP_Show();//APP数据显示
		delay_ms(50);
	}
}
 

















