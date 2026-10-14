#include "TrackModule.h" 
/*=============================================================================
 * 可调参数区域
 *=============================================================================*/
// 转向角度参数
float Turn90Angle  = 101;   // 直角弯转向参数
float TurnMaxAngle = 60;   // 大弯道转向参数
float TurnMidAngle = 17;   // 中等转向参数（丢线时使用）
float TurnMinAngle = 7;   // 微调转向参数
extern int temp;
// 速度参数
float BaseSpeed = 250;      // 基础巡线速度（直行时的速度）
float ForwardLimit = 788;		//前行限制(转向大于该值限制其前进)
/*=============================================================================
 * 传感器状态定义--识别到黑线时为1
 *=============================================================================*/
typedef enum {
    STATE_CROSS         = 0,    // 0000 - 十字路口（全黑）
    STATE_LEFT_90_A     = 1,    // 0001 - 左直角弯
	STATE_LEFT_90_B		= 3,	// 0011
    STATE_RIGHT_90_A    = 8,  	// 1000 - 右直角弯
	STATE_RIGHT_90_B    = 12,	// 1100
    STATE_LEFT_BIG      = 7,    // 0111 - 左大弯
    STATE_RIGHT_BIG     = 14,   // 1110 - 右大弯
    STATE_LEFT_SMALL    = 11,   // 1011 - 左微调
    STATE_RIGHT_SMALL   = 13,   // 1101 - 右微调
    STATE_STRAIGHT      = 9,    // 1001 - 直行
    STATE_LOST          = 15    // 1111 - 丢线（全白）
} SensorState_t;
float base_speed_mm = 0;        // 基础速度（mm/s）
float turn_diff = 0;            // 转向差速
/*=============================================================================
 * 巡线功能函数（输出两电机目标速度）
 *=============================================================================*/
void IRDM_line_inspection(void)
{
    static int last_state = 0;      // 记录上一次的状态
	float left_motor_speed = 0;     // 左电机临时速度（m/s）
    float right_motor_speed = 0;    // 右电机临时速度（m/s）
	static int turn_cnt=0;
	static int saved_state = 0;  // 保存转向状态
    // 读取传感器状态：4个传感器组合值
    int sensor_state = (DH1 << 3) | (DH2 << 2) | (DH3 << 1) | DH4;
  /*=========================================================================*
     * 状态判断：设置转向差速												   *
     *=========================================================================*/
    switch (sensor_state)
    {
        case STATE_CROSS:// 交叉路口处理
			turn_diff = 0;
            break;
        case STATE_LEFT_90_A: // 左直角弯
		case STATE_LEFT_90_B: // 左直角弯
            turn_diff = Turn90Angle;
            break;
        case STATE_RIGHT_90_A: // 右直角弯
		case STATE_RIGHT_90_B: // 右直角弯
            turn_diff = -Turn90Angle;
            break;
        case STATE_LEFT_BIG://左大弯
            turn_diff = TurnMaxAngle;
            break;
        case STATE_RIGHT_BIG://右大弯
            turn_diff = -TurnMaxAngle;
            break;
        case STATE_LEFT_SMALL://左微调
            turn_diff = TurnMinAngle;
            break;
        case STATE_RIGHT_SMALL://右微调
            turn_diff = -TurnMinAngle;
            break;
        case STATE_STRAIGHT://直行
            turn_diff = 0;
            break;
        case STATE_LOST://丢线处理
            if (last_state == STATE_LEFT_SMALL) turn_diff = TurnMidAngle;//继续左转
			else if (last_state == STATE_RIGHT_SMALL) turn_diff = -TurnMidAngle;//继续右转
			else if(last_state == STATE_LEFT_BIG ) turn_diff = TurnMaxAngle;//继续左转
			else if(last_state == STATE_RIGHT_BIG ) turn_diff = -TurnMaxAngle;//继续右转
            break;
        default: // 未定义状态，直行
            turn_diff = 0;
            break;
    }
	//保存传感器状态
	if(sensor_state!=STATE_LOST)
	{
		last_state=sensor_state;
	}
    // 转向速度越大，基础速度越低
	if(fabs(turn_diff)<ForwardLimit)
	{
		base_speed_mm = BaseSpeed - (BaseSpeed * (fabs(turn_diff) / ForwardLimit));
	}
	else base_speed_mm=0;
    /*========================================================================*
     * 设置电机目标速度（左-转向差速，右+转向差速，单位：mm/s）                   *
     *=========================================================================*/
	left_motor_speed = 0.001f * (base_speed_mm - turn_diff); 
    right_motor_speed = 0.001f * (base_speed_mm + turn_diff);
    // 赋值给电机目标速度
    MOTOR_A.Target = left_motor_speed;//左电机
    MOTOR_B.Target = right_motor_speed;//右电机
}


void TrackModule_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//Enable the ability port clock
	//使能端口时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_12;			//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		//普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		//下拉
	GPIO_Init(GPIOE, &GPIO_InitStructure);	
	
}
/*=============================================================================
 * 巡线功能函数（输出两电机目标速度）

//C30DV2.0巡线例程
 *=============================================================================*/
void IRDM_line_inspection_V2(void)
{
    static int last_state = 0;      // 记录上一次的状态
	float left_motor_speed = 0;     // 左电机临时速度（m/s）
    float right_motor_speed = 0;    // 右电机临时速度（m/s）
    static int turn_cnt = 0;         // 转向阶段计数器
	static int saved_state = 0;  // 保存转向状态
    // 读取传感器状态：4个传感器组合值
    int sensor_state = (DH1_V2 << 3) | (DH2_V2 << 2) | (DH3_V2 << 1) | DH4_V2; // 将传感器状态组合成一个整数
   /*=========================================================================*
     * 状态判断：设置转向差速												   *
     *=========================================================================*/
    switch (sensor_state)
    {
        case STATE_CROSS:// 交叉路口处理
			turn_diff = 0;
			last_state = STATE_CROSS;
            break;
              case STATE_LEFT_90_A: // 左直角弯
		case STATE_LEFT_90_B: // 左直角弯
            turn_diff = Turn90Angle;
            last_state = STATE_LEFT_90_A;
            break;
        case STATE_RIGHT_90_A: // 右直角弯
		case STATE_RIGHT_90_B: // 右直角弯
            turn_diff = -Turn90Angle;
            last_state = STATE_RIGHT_90_A;
            break;
        case STATE_LEFT_BIG://左大弯
            turn_diff = TurnMaxAngle;
            last_state = STATE_LEFT_BIG;
            break;
        case STATE_RIGHT_BIG://右大弯
            turn_diff = -TurnMaxAngle;
            last_state = STATE_RIGHT_BIG;
            break;
        case STATE_LEFT_SMALL://左微调
            turn_diff = TurnMinAngle;
            last_state = STATE_LEFT_SMALL;
            break;
        case STATE_RIGHT_SMALL://右微调
            turn_diff = -TurnMinAngle;
            last_state = STATE_RIGHT_SMALL;
            break;
        case STATE_STRAIGHT://直行
            turn_diff = 0;
            last_state = STATE_STRAIGHT;
            break;
        case STATE_LOST://丢线处理
            if (last_state == STATE_LEFT_SMALL) turn_diff = TurnMidAngle;//继续左转
			else if (last_state == STATE_RIGHT_SMALL) turn_diff = -TurnMidAngle;//继续右转
			else if(last_state == STATE_LEFT_BIG ) turn_diff = TurnMaxAngle;//继续左转
			else if(last_state == STATE_RIGHT_BIG ) turn_diff = -TurnMaxAngle;//继续右转
            break;
        default: // 未定义状态，直行
            turn_diff = 0;
            last_state = STATE_STRAIGHT;
            break;
    }
    // 转向速度越大，基础速度越低
	if(fabs(turn_diff)<ForwardLimit)
	{
		base_speed_mm = BaseSpeed - (BaseSpeed * (fabs(turn_diff) / ForwardLimit));
	}
	else base_speed_mm=0;
    /*========================================================================*
     * 设置电机目标速度（左-转向差速，右+转向差速，单位：mm/s）                   *
     *=========================================================================*/
	left_motor_speed = 0.001f * (base_speed_mm - turn_diff); 
    right_motor_speed = 0.001f * (base_speed_mm + turn_diff);
    // 赋值给电机目标速度
    MOTOR_A.Target = left_motor_speed;//左电机
    MOTOR_B.Target = right_motor_speed;//右电机
}

void TrackModule_Init_V2(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//Enable the ability port clock
	//使能端口时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;			//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		//普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		//下拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;			//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		//普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		//下拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;			//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		//普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		//下拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
}











