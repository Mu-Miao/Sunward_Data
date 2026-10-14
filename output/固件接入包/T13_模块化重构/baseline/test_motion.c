
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "dual_pi_test.h"
#include "lab_uart_async.h"
#include "mpu6050.h"
DWT_Type mock_dwt;CoreDebug_Type mock_core;
static uint32_t ms,tick,bat=1320,io_us=3000;
static int off=1,adc_fail,imu_fail,off_during_read,full_tx;
static int led_on, led1, led2, beep_on, signals, key_input;
static unsigned line_inputs=15, line_reads;
static char oled_lines[4][32];
static int a,b;static int16_t gx,gy,gz=-424;
static char rx[2][1024],logs[262144];static unsigned rp[2],wp[2];static int faults[2];
static TIM_HandleTypeDef l,r;static ADC_HandleTypeDef adc_mock;static UART_HandleTypeDef uart_mock;
uint32_t HAL_GetTick(void){return ms;}
uint32_t HAL_RCC_GetHCLKFreq(void){return 72000000U;}
void HAL_Delay(uint32_t n){ms+=n;DWT->CYCCNT+=n*72000U;}
int HAL_GPIO_ReadPin(void*p,uint32_t pin){
 if(p==GPIOA){assert(pin==GPIO_PIN_0);return key_input;}
 if(p==GPIOC&&pin==GPIO_PIN_8){++line_reads;return (line_inputs>>0)&1U;}
 if(p==GPIOC&&pin==GPIO_PIN_4){++line_reads;return (line_inputs>>1)&1U;}
 if(p==GPIOC&&pin==GPIO_PIN_9){++line_reads;return (line_inputs>>2)&1U;}
 if(p==GPIOB&&pin==GPIO_PIN_8){++line_reads;return (line_inputs>>3)&1U;}
 assert(p==GPIOC&&pin==GPIO_PIN_13);return off;
}
void HAL_GPIO_WritePin(void*p,uint32_t pin,int value){
 if(p==GPIOC){
  assert(pin==GPIO_PIN_2||pin==(GPIO_PIN_2|GPIO_PIN_3));
  led1=value==GPIO_PIN_RESET;
  if(pin&GPIO_PIN_3)led2=value==GPIO_PIN_RESET;
  led_on=led1&&led2;
 }
 else{assert(p==GPIOA&&pin==GPIO_PIN_15);beep_on=value==GPIO_PIN_SET;if(beep_on)signals++;}
}
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef*p){(void)p;return adc_fail?HAL_ERROR:HAL_OK;}
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef*p,uint32_t n){(void)p;(void)n;return adc_fail?HAL_ERROR:HAL_OK;}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef*p){(void)p;return bat;}
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef*p){(void)p;return HAL_OK;}
HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_Raw*p){
 DWT->CYCCNT+=io_us*72U;if(off_during_read)off=1;
 if(imu_fail)return HAL_ERROR;
 *p=(MPU6050_Raw){.gx=gx,.gy=gy,.gz=gz};return HAL_OK;
}
HAL_StatusTypeDef LabUART_Begin(UART_HandleTypeDef*p){(void)p;return HAL_OK;}
HAL_StatusTypeDef LabUART_BeginPort(LabUART_Port c,UART_HandleTypeDef*p){(void)c;(void)p;return HAL_OK;}
int LabUART_TryWritePort(LabUART_Port c,const char*s,uint16_t n){
 (void)c;if(full_tx)return 0;assert(strlen(logs)+n<sizeof logs);strncat(logs,s,n);return 1;
}
int LabUART_TryWrite(const char*s,uint16_t n){return LabUART_TryWritePort(LAB_UART_USB,s,n);}
int LabUART_TryReadPort(LabUART_Port c,uint8_t*p){if(rp[c]==wp[c])return 0;*p=(uint8_t)rx[c][rp[c]++];return 1;}
int LabUART_TakeRxFaultPort(LabUART_Port c){int n=faults[c];faults[c]=0;return n;}
uint32_t LabUART_RxErrors(void){return 0;}uint32_t LabUART_TxDropped(void){return full_tx?1:0;}
uint32_t LabUART_RxErrorsPort(LabUART_Port c){(void)c;return 0;}
uint32_t LabUART_TxDroppedPort(LabUART_Port c){(void)c;return full_tx?1:0;}
static void motor(int aa,int bb){assert(aa>=-3000&&aa<=3000&&bb>=-3000&&bb<=3000);a=aa;b=bb;}
void TaskOLED_Begin(uint32_t n){assert(n==72000000U);}
int TaskOLED_SetLines(const char *const text[4]){
 for(unsigned i=0;i<4;++i){assert(strlen(text[i])<sizeof oled_lines[i]);strcpy(oled_lines[i],text[i]);}return 1;
}
void TaskOLED_Poll(uint32_t n){(void)n;}
uint32_t TaskOLED_LastIOUs(void){return 100U;}
#include "dual_pi_test.c"
static void setup_key(int held){
 ms=tick=0;memset(&mock_dwt,0,sizeof mock_dwt);memset(&mock_core,0,sizeof mock_core);
 l.count=r.count=0;memset(rx,0,sizeof rx);memset(rp,0,sizeof rp);memset(wp,0,sizeof wp);memset(faults,0,sizeof faults);
 logs[0]=0;signals=led_on=led1=led2=beep_on=0;key_input=held;off=1;adc_fail=imu_fail=off_during_read=full_tx=0;io_us=3000;bat=1320;gx=gy=0;gz=-424;
 assert(DualPITest_Begin(&uart_mock,&l,&r,&adc_mock,motor)==HAL_OK);
 assert(DualPITest_EnableBluetooth(&uart_mock)==HAL_OK);DualPITest_EnableHeading(1);
 DualPITest_Poll(tick);
}
static void setup(void){setup_key(0);}
static void step(unsigned n,int dl,int dr){
 /* Each call is scheduled from the preceding sample start, not I2C completion. */
 ms+=n;DWT->CYCCNT=old_cycles+n*72000U;tick+=n/10U;
 l.count=(l.count+dl)&65535U;r.count=(r.count-dr)&65535U;
 DualPITest_Poll(tick);
}
static void queue(unsigned c,const char*s){size_t n=strlen(s);assert(wp[c]+n<sizeof rx[c]);memcpy(rx[c]+wp[c],s,n);wp[c]+=(unsigned)n;}
static void cmd(const char*s){queue(0,s);DualPITest_Poll(tick);}
static void enable(void){off=0;for(int i=0;i<20;i++)step(10,0,0);}
static void calibrate(void){cmd("CAL!");for(int i=0;i<705;i++)step(10,0,0);assert(yaw_phase==Y_READY&&yaw_samples==500&&fabsf(yaw_bias+424)<.001f&&a==0&&b==0);}
static void head(void){enable();cmd("GO!");step(10,0,0);assert(state==RUNNING&&motion==MOVE_HEADING&&a==1800&&b==-1800);}
static void release_key(void){key_input=0;for(int i=0;i<4;i++)step(10,0,0);}
static void press_key(void){key_input=1;for(int i=0;i<4;i++)step(10,0,0);}
int main(void){
 setup();enable();cmd("GO!");step(10,0,0);assert(a==0&&b==0&&state!=RUNNING);cmd("CAL!");assert(yaw_phase==Y_NONE);
 setup();cmd("CAL!GO!");for(int i=0;i<705;i++)step(10,0,0);assert(yaw_phase==Y_READY&&state!=RUNNING);enable();for(int i=0;i<5;i++)step(10,0,0);assert(a==0&&b==0);
 setup();calibrate();head();gz=886;for(int i=0;i<10;i++)step(10,150,150);assert(yaw_error<0&&targets[LEFT]>60&&targets[RIGHT]<60);cmd("STOP!");assert(state==STOPPED&&a==0&&b==0&&wheels[0].integral==0&&wheels[1].integral==0);
 setup();calibrate();head();gz=-1734;for(int i=0;i<10;i++)step(10,150,150);assert(yaw_error>0&&targets[LEFT]<60&&targets[RIGHT]>60);
 setup();calibrate();head();gz=25776;for(int i=0;i<8;i++)step(10,150,150);assert(state==RUNNING&&fabsf(yaw_diff+15)<.01f&&targets[LEFT]==75&&targets[RIGHT]==45);for(int i=0;i<5;i++)step(10,150,150);assert(state==YAW_BAD&&a==0&&b==0);
 setup();calibrate();head();imu_fail=1;step(10,150,150);assert(state==YAW_BAD&&a==0&&b==0);
 setup();calibrate();head();io_us=6000;step(10,150,150);assert(state==YAW_BAD&&a==0&&b==0);
 setup();calibrate();head();gz=32760;step(10,150,150);assert(state==YAW_BAD&&a==0&&b==0);
 setup();calibrate();head();off_during_read=1;step(10,150,150);assert(state==OFF&&a==0&&b==0);
 setup();calibrate();head();for(int i=0;i<150;i++)step(10,150,150);cmd("GO!");for(int i=0;i<149;i++)step(10,150,150);assert(state==DONE&&a==0&&b==0);
 setup();calibrate();head();queue(0,"GO!");queue(1,"STOP!");DualPITest_Poll(tick);assert(state==STOPPED&&a==0&&b==0);
 setup();calibrate();head();queue(0,"STOP!");queue(1,"GO!");DualPITest_Poll(tick);assert(state==STOPPED&&a==0&&b==0);
 setup();calibrate();head();faults[1]=1;DualPITest_Poll(tick);assert(state==UART_BAD&&a==0&&b==0);
 setup();calibrate();head();step(20,150,150);assert(state==GAP&&a==0&&b==0);
 setup();cmd("CAL!");for(int i=0;i<205;i++)step(10,0,0);step(10,1,0);assert(yaw_phase==Y_FAIL&&a==0&&b==0);
 setup();cmd("CAL!");for(int i=0;i<205;i++)step(10,0,0);off=0;step(10,0,0);assert(yaw_phase==Y_FAIL&&a==0&&b==0);
 setup();cmd("CAL!");for(int i=0;i<205;i++)step(10,0,0);gz=1000;step(10,0,0);gz=-424;for(int i=0;i<500;i++)step(10,0,0);assert(yaw_phase==Y_FAIL);
 setup();cmd("CAL!");cmd("STOP!");for(int i=0;i<705;i++)step(10,0,0);assert(yaw_phase==Y_NONE&&a==0&&b==0);
 setup();queue(0,"CAL!");queue(1,"STOP!");DualPITest_Poll(tick);assert(yaw_phase==Y_NONE);
 setup();calibrate();enable();cmd("F!");step(10,0,0);assert(a==3000&&b==-3000);for(int i=0;i<20;i++)step(10,300,300);cmd("GO!");assert(motion==MOVE_FORWARD);cmd("STOP!");
 /* Existing commands work without successful MPU calibration. */
 setup();DualPITest_EnableHeading(0);enable();cmd("RUN!");step(10,0,0);assert(a==1800&&b==-1800);for(int i=0;i<299;i++)step(10,150,150);assert(state==DONE);
 setup();enable();cmd("B!");step(10,0,0);assert(a==-3000&&b==3000);for(int i=0;i<20;i++)step(10,-300,-300);assert(state==RUNNING);cmd("S!");assert(state==STOPPED);
 setup();enable();cmd("L!");step(10,0,0);assert(targets[LEFT]==60&&targets[RIGHT]==120);cmd("R!");assert(a==0&&b==0);for(int i=0;i<20;i++)step(10,0,0);assert(targets[LEFT]==120&&targets[RIGHT]==60&&a>0&&b<0);cmd("STOP!");
 setup();enable();cmd("V150!");cmd("TIME3!");cmd("F!");step(10,0,0);assert(targets[0]==150&&run_duration==3000);cmd("V200!");assert(manual_speed==150);cmd("STOP!");
 setup();calibrate();head();full_tx=1;for(int i=0;i<25;i++)step(10,150,150);assert(state==RUNNING);cmd("STOP!");assert(a==0&&b==0);
 setup();calibrate();head();bat=1100;for(int i=0;i<10;i++)step(10,150,150);assert(state==LOW&&a==0&&b==0);
 setup();calibrate();head();adc_fail=1;for(int i=0;i<10;i++)step(10,150,150);assert(state==ADC_BAD&&a==0&&b==0);
 setup();calibrate();head();for(int i=0;i<60;i++)step(10,0,0);assert(state==STALL&&a==0&&b==0);
 setup();calibrate();head();for(int i=0;i<5;i++)step(10,-40,150);assert(state==REVERSE&&a==0&&b==0);
 setup();calibrate();head();step(10,1100,150);assert(state==FAST&&a==0&&b==0);
 /* Calibration and reference survive DWT wrap; heading shortest-angle error. */
 assert(fabsf(wrap_degrees(-358)-2)<.001f);assert(fabsf(wrap_degrees(358)+2)<.001f);
 setup();calibrate();DWT->CYCCNT=old_cycles=0xffff0000U;yaw_cycles=old_cycles+io_us*72U;head();step(10,150,150);assert(state==RUNNING);cmd("STOP!");

 /* Distance task uses encoder travel and settles before DONE. */
 setup();enable();cmd("GO20!");step(10,0,0);assert(state!=RUNNING&&a==0&&b==0);
 setup();calibrate();enable();cmd("GO20!");step(10,0,0);
 assert(state==RUNNING&&motion==MOVE_DISTANCE&&run_duration==10000&&distance_mm[0]==0&&distance_mm[1]==0);
 int slowed=0,floor_seen=0;
 for(int i=0;i<400&&state==RUNNING;i++){
  step(10,176,176);
  if(distance_base<59.0f)slowed=1;
  if(distance_base==40.0f)floor_seen=1;
  assert(distance_base>=40.0f&&distance_base<=60.0f);
 }
 assert(state==BRAKING&&slowed&&floor_seen&&a==0&&b==0);
 assert((distance_mm[0]+distance_mm[1])*.5f>=200.0f&&(distance_mm[0]+distance_mm[1])*.5f<200.7f);
 assert(wheels[0].integral==0&&wheels[1].integral==0);
 for(int i=0;i<5;i++)step(10,30,30); /* coast is included in final mileage */
 float coast=(distance_mm[0]+distance_mm[1])*.5f;
 for(int i=0;i<20;i++)step(10,0,0);assert(state==BRAKING);
 step(10,0,0);assert(state==DONE&&fabsf((distance_mm[0]+distance_mm[1])*.5f-coast)<.001f);
 float final_distance=distance_mm[0];for(int i=0;i<30;i++)step(10,20,20);
 assert(fabsf(distance_mm[0]-final_distance)<.001f&&strstr(logs,"M=DIST")&&strstr(logs," G=200"));
 cmd("GO20!");assert(state==RUNNING&&distance_mm[0]==0&&distance_mm[1]==0);cmd("STOP!");assert(state==STOPPED&&distance_tracking==0&&a==0&&b==0);
 /* Waiting for settle never accepts a movement command. */
 setup();calibrate();enable();cmd("GO20!");
 for(int i=0;i<400&&state==RUNNING;i++)step(10,176,176);assert(state==BRAKING);
 cmd("F!");cmd("GO20!");assert(state==BRAKING&&a==0&&b==0);cmd("STOP!");assert(state==STOPPED&&!distance_tracking);
 /* Braking timeout is a fault, not successful completion. */
 setup();calibrate();enable();cmd("GO20!");
 for(int i=0;i<400&&state==RUNNING;i++)step(10,176,176);
 for(int i=0;i<150;i++)step(10,30,30);assert(state==SETTLE_BAD&&a==0&&b==0&&!distance_tracking);
 /* 3 seconds and repeated commands don't finish or reset distance. */
 setup();calibrate();enable();cmd("GO20!");for(int i=0;i<305;i++)step(10,150,150);
 assert(state==RUNNING&&(distance_mm[0]+distance_mm[1])*.5f<200);
 float before_distance=distance_mm[0];uint32_t before_start=start_ms;cmd("GO20!");
 assert(distance_mm[0]==before_distance&&start_ms==before_start);
 /* Slow progress reaches the fixed safety deadline. */
 setup();calibrate();enable();cmd("GO20!");for(int i=0;i<999;i++)step(10,20,20);assert(state==RUNNING);
 cmd("GO20!");step(10,20,20);assert(state==TIMEOUT&&a==0&&b==0&&!distance_tracking);
 /* Signed mean, separate CPRs, and pre-start motion exclusion. */
 setup();calibrate();enable();l.count+=100;r.count=(r.count-100)&65535;cmd("GO20!");step(10,0,0);
 assert(distance_mm[0]==0&&distance_mm[1]==0);step(10,176,150);
 assert(fabsf(distance_mm[0]-176*DPT_CIRC_M*1000/DPT_LEFT_CPR)<.0001f);
 assert(fabsf(distance_mm[1]-150*DPT_CIRC_M*1000/DPT_RIGHT_CPR)<.0001f);
 float forward_left=distance_mm[0];step(10,-30,150);assert(distance_mm[0]<forward_left);cmd("STOP!");
 /* Encoder counter rollover and time rollover retain the same distance task. */
 setup();calibrate();enable();l.count=65500;r.count=40;wheels[0].old_count=65500;wheels[1].old_count=40;
 cmd("GO20!");for(int i=0;i<400&&state==RUNNING;i++)step(10,176,176);assert(state==BRAKING);
 setup();calibrate();enable();ms=0xfffffff0U;on_ms=ms-200U;adc_ms=ms;log_ms=yaw_log_ms=ms;old_ms=ms;
 cmd("GO20!");for(int i=0;i<400&&state==RUNNING;i++)step(10,176,176);assert(state==BRAKING);
 for(int i=0;i<21;i++)step(10,0,0);assert(state==DONE);
 /* Existing guards and UART priority apply to distance motion and braking. */
 setup();calibrate();enable();queue(0,"GO20!");queue(1,"STOP!");DualPITest_Poll(tick);assert(state==STOPPED);
 setup();calibrate();enable();cmd("GO20!");step(10,176,176);off=1;DualPITest_Poll(tick);assert(state==OFF&&a==0&&b==0);
 setup();calibrate();enable();cmd("GO20!");step(10,1100,150);assert(state==FAST&&a==0&&b==0&&!distance_tracking);
 setup();calibrate();enable();cmd("GO20!");step(20,176,176);assert(state==GAP&&distance_mm[0]==0);
 setup();calibrate();enable();cmd("GO20!");step(10,176,176);imu_fail=1;step(10,176,176);assert(state==YAW_BAD&&a==0&&b==0);
 setup();calibrate();enable();cmd("GO20!");full_tx=1;for(int i=0;i<40;i++)step(10,176,176);cmd("STOP!");assert(state==STOPPED&&a==0&&b==0);
 setup();calibrate();enable();cmd("GO20!");for(int i=0;i<400&&state==RUNNING;i++)step(10,176,176);off=1;DualPITest_Poll(tick);assert(state==OFF&&!distance_tracking);
 setup();calibrate();enable();cmd("GO20!");for(int i=0;i<400&&state==RUNNING;i++)step(10,176,176);step(20,0,0);assert(state==GAP&&!distance_tracking);
 setup();calibrate();enable();cmd("GO20!");for(int i=0;i<400&&state==RUNNING;i++)step(10,176,176);faults[1]=1;DualPITest_Poll(tick);assert(state==UART_BAD&&!distance_tracking);
 setup();calibrate();enable();cmd("GO20!");for(int i=0;i<400&&state==RUNNING;i++)step(10,176,176);imu_fail=1;step(10,0,0);assert(state==YAW_BAD&&!distance_tracking);
 setup();calibrate();enable();cmd("GO20!");for(int i=0;i<400&&state==RUNNING;i++)step(10,176,176);
 step(10,0,0);step(10,0,0);step(10,30,30);for(int i=0;i<20;i++)step(10,0,0);assert(state==BRAKING);step(10,0,0);assert(state==DONE);

 /* 1m route keeps its goal, speed and deadline separate from GO20/manual configuration. */
 setup();enable();cmd("GO100!");assert(state!=RUNNING&&!led_on&&!beep_on);
 setup();calibrate();enable();cmd("V200!TIME1!GO100!");
 assert(state==RUNNING&&motion==MOVE_ROUTE&&distance_goal==1000&&distance_cruise==120&&distance_slow==100&&run_duration==13500);
 assert(targets[0]==40&&targets[1]==40&&!completion_signal&&!signals);
 step(10,120,120);assert(distance_base>40&&distance_base<=41.1f);
 for(int i=0;i<40;i++)step(10,300,300);
 assert(distance_base>80&&distance_base<83&&distance_mm[0]>40);
 float travelled=distance_mm[0];uint32_t started=start_ms;
 cmd("GO20!GO100!");assert(distance_mm[0]==travelled&&distance_goal==1000&&start_ms==started);
 for(int i=0;i<40;i++)step(10,300,300);assert(distance_base==120);
 gz=886;step(10,300,300);assert(targets[0]>targets[1]);gz=-424;
 int slow_seen=0, floor_seen_1m=0;
 for(int i=0;i<1100&&state==RUNNING;i++){
  step(10,300,300);
  if(distance_mm[0]>905&&distance_base<115)slow_seen=1;
  if(distance_mm[0]>970&&distance_base==40)floor_seen_1m=1;
  assert(targets[0]>=0&&targets[1]>=0&&fabsf(yaw_diff)<=15.01f);
 }
 assert(state==BRAKING&&slow_seen&&floor_seen_1m&&a==0&&b==0&&!signals);
 for(int i=0;i<5;i++)step(10,30,30);assert(!signals);
 for(int i=0;i<21;i++)step(10,0,0);
 assert(state==DONE&&signals==1&&completion_signal&&led_on&&beep_on&&a==0&&b==0);
 float stopped_at=distance_mm[0];
 for(int i=0;i<49;i++)step(10,0,0);assert(completion_signal&&signals==1);
 step(10,0,0);assert(!completion_signal&&!led_on&&!beep_on&&signals==1&&distance_mm[0]==stopped_at);
 assert(strstr(logs," G=1000 SIG=1")&&strstr(logs," G=1000 SIG=0"));
 for(int i=0;i<50;i++)step(10,0,0);assert(signals==1&&state==DONE);
 cmd("GO20!");assert(state==RUNNING&&motion==MOVE_DISTANCE&&distance_goal==200&&distance_cruise==60&&distance_slow==40&&run_duration==10000&&targets[0]==60);
 cmd("STOP!");assert(!signals||signals==1);assert(!completion_signal);
 /* Partial and unknown commands cannot start a 1m movement. */
 setup();calibrate();enable();cmd("GO10!GO1000!GO0100!");assert(state!=RUNNING);
 queue(0,"GO");queue(1,"100!");DualPITest_Poll(tick);assert(state!=RUNNING);
 /* Deadline is a fault, never a completion signal. */
 setup();calibrate();enable();cmd("GO100!");
 for(int i=0;i<1349;i++)step(10,20,20);assert(state==RUNNING);
 cmd("GO100!");step(10,20,20);assert(state==TIMEOUT&&!signals&&!completion_signal&&a==0&&b==0);
 /* STOP on either input wins over GO100. */
 for(int port=0;port<2;port++){
  setup();calibrate();enable();queue((unsigned)port,"GO100!");queue((unsigned)(1-port),"STOP!");DualPITest_Poll(tick);
  assert(state==STOPPED&&!signals&&a==0&&b==0);
 }
 /* Route safety gates apply in RUN and BRAKE, with no false DONE or sound/light. */
 for(int brake=0;brake<2;brake++)for(int fault=0;fault<7;fault++){
  setup();calibrate();enable();cmd("GO100!");
  if(brake){for(int i=0;i<1100&&state==RUNNING;i++)step(10,300,300);assert(state==BRAKING);}
  if(fault==0){off=1;DualPITest_Poll(tick);assert(state==OFF);}
  if(fault==1){cmd("STOP!");assert(state==STOPPED);}
  if(fault==2){faults[1]=1;DualPITest_Poll(tick);assert(state==UART_BAD);}
  if(fault==3){imu_fail=1;step(10,0,0);assert(state==YAW_BAD);}
  if(fault==4){step(20,0,0);assert(state==GAP);}
  if(fault==5){bat=1100;for(int i=0;i<10;i++)step(10,20,20);assert(state==LOW);}
  if(fault==6){adc_fail=1;for(int i=0;i<10;i++)step(10,20,20);assert(state==ADC_BAD);}
  assert(!completion_signal&&!signals&&a==0&&b==0&&!distance_tracking);
 }
 setup();calibrate();enable();cmd("GO100!");
 for(int i=0;i<1100&&state==RUNNING;i++)step(10,300,300);
 cmd("GO20!GO100!F!");assert(state==BRAKING&&motion==MOVE_ROUTE);
 for(int i=0;i<150;i++)step(10,30,30);assert(state==SETTLE_BAD&&!signals);
 /* A stopped indication is cancelled promptly by STOP/OFF/recalibration or new motion. */
 for(int action=0;action<4;action++){
  setup();calibrate();enable();cmd("GO100!");
  for(int i=0;i<1100&&state==RUNNING;i++)step(10,300,300);
  for(int i=0;i<21;i++)step(10,0,0);assert(completion_signal&&signals==1);
  if(action==0)cmd("STOP!");
  if(action==1){off=1;DualPITest_Poll(tick);}
  if(action==2)cmd("GO20!");
  if(action==3){off=1;cmd("CAL!");assert(yaw_phase==Y_WARM);}
  assert(!completion_signal&&!led_on&&!beep_on&&signals==1);
 }
 /* Timer wrap is valid for the 1m deadline and the once-only 500ms signal. */
 setup();calibrate();enable();ms=0xfffffff0U;on_ms=ms-200U;adc_ms=ms;log_ms=yaw_log_ms=ms;old_ms=ms;
 cmd("GO100!");step(10,120,120);assert(state==RUNNING&&distance_base<=41.1f);
 for(int i=0;i<1100&&state==RUNNING;i++)step(10,300,300);assert(state==BRAKING);
 for(int i=0;i<21;i++)step(10,0,0);assert(signals==1&&completion_signal);
 ms=0xfffffff0U;signal_ms=ms;old_ms=ms;adc_ms=ms;log_ms=yaw_log_ms=ms;
 for(int i=0;i<50;i++)step(10,0,0);assert(!completion_signal&&signals==1);
 /* A blocked transmit queue cannot delay stopping or ending the indication. */
 setup();calibrate();enable();cmd("GO100!");
 for(int i=0;i<1100&&state==RUNNING;i++)step(10,300,300);
 for(int i=0;i<21;i++)step(10,0,0);full_tx=1;
 for(int i=0;i<50;i++)step(10,0,0);assert(!completion_signal&&signals==1&&a==0&&b==0);


 setup();calibrate();enable();press_key();assert(state==RUNNING&&motion==MOVE_ROUTE);
 release_key();press_key();assert(state==STOPPED&&a==0&&b==0);
 puts("PASS: prior serial GO100/GO20/heading/manual motion regression, timing/MPU/ADC/encoder/UART protections, braking/coast/settle/completion and unchanged ON key start/busy STOP.");
}
