#ifndef __TRACKMODULE_H
#define __TRACKMODULE_H
#include "sys.h"


#define DH1 PEin(8)
#define DH2 PEin(10)
#define DH3 PEin(12)
#define DH4 PEin(15)	

#define DH1_V2 PAin(6)
#define DH2_V2 PCin(5)
#define DH3_V2 PDin(15)
#define DH4_V2 PAin(7)	


extern float Turn90Angle ;   // 直角弯转向参数
extern float TurnMaxAngle;   // 大弯道转向参数
extern float TurnMidAngle;   // 中等转向参数（丢线时使用）
extern float TurnMinAngle;   // 微调转向参数

void TrackModule_Init(void);
void IRDM_line_inspection(void);
void TrackModule_Init_V2(void);
void IRDM_line_inspection_V2(void);
extern float base_speed_mm ,BaseSpeed,ForwardLimit;// 基础速度（mm/s）
extern float turn_diff ;    // 转向差速
#endif

