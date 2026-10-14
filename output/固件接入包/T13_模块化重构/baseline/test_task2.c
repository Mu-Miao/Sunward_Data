
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
 DualPITest_EnableHeading(1);
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

static float progress(void){return (distance_mm[0]+distance_mm[1])*.5f-task2_leg_start;}
static void hold(void){
 release_key();key_input=1;for(int i=0;i<154;i++)step(10,0,0);release_key();
 assert(a==0&&b==0&&key_action==KEY_MODE&&yaw_phase!=Y_WARM&&yaw_phase!=Y_CAL);
}
static void task_start(void){
 setup();line_inputs=15;hold();assert(key_motion==MOVE_LINE);
 hold();assert(key_motion==MOVE_TASK2);release_key();press_key();release_key();
 for(int i=0;i<705;i++)step(10,0,0);
 assert(yaw_phase==Y_READY);enable();press_key();
 assert(state==RUNNING&&motion==MOVE_TASK2&&task2_leg==0&&!bluetooth_on);
 assert(wp[0]==0&&wp[1]==0&&distance_mm[0]==0&&task2_elapsed_ms==0);
}
static void endpoint(unsigned leg){
 assert(task2_leg==leg&&state==RUNNING);
 float goal=leg%2?DPT_TASK2_ARC_MM:DPT_TASK2_STRAIGHT_MM;
 while(state==RUNNING&&task2_leg==leg){
  float prog=progress();
  if(leg%2){
   yaw_angle=wrap_degrees(task2_start_heading-180.0f*(float)(leg/2+prog/DPT_TASK2_ARC_MM));
   line_inputs=prog>=goal?15:9;
  }else{
   yaw_angle=leg==0?task2_start_heading:wrap_degrees(task2_start_heading+180);
   line_inputs=prog>=goal?9:15;
  }
  step(10,610,610);
  if(task2_edge_seen&&(leg%2))assert(a==0&&b==0&&targets[0]==0&&targets[1]==0);
 }
 assert(leg==3?state==BRAKING:state==RUNNING);
 assert(task2_leg==(leg==3?3:leg+1));
}
static void fast_to_arc(void){task_start();endpoint(0);assert(task2_arc());}
static void near_arc_end(void){
 fast_to_arc();distance_mm[0]=distance_mm[1]=task2_leg_start+DPT_TASK2_ARC_MM-10;
 yaw_angle=wrap_degrees(task2_start_heading+180);
}
int main(void){
 /* Independent raw-pattern truth table retains the tuned LF polarity and signs. */
 const int valid[16]={0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0};
 const float left_at200[16]={0,136,0,120,0,0,0,120,264,200,0,136,280,264,280,0};
 for(unsigned mask=0;mask<16;mask++){
  setup();line_inputs=mask;line_decode();assert(line_valid==valid[mask]);
  if(valid[mask]){
   line_targets(ms,200);assert(fabsf(targets[0]-left_at200[mask])<.001f);
   assert(fabsf(targets[1]-(400-left_at200[mask]))<.001f);
   line_targets(ms,50);assert(targets[0]>=30&&targets[1]>=30&&targets[0]<=70&&targets[1]<=70);
  }
 }
 /* Reported BC exit S1256: new endpoint window hands off to CD instead of LOST. */
 fast_to_arc();distance_mm[0]=distance_mm[1]=task2_leg_start+1256.0f;
 yaw_angle=wrap_degrees(task2_start_heading+180.0f);line_inputs=15;
 for(int i=0;i<4;i++)step(10,0,0);
 assert(state==RUNNING&&task2_leg==2&&signals==2);
 /* Full GPIO-only route crosses white straights and black arcs exactly once. */
 task_start();uint32_t began=start_ms;
 endpoint(0);assert(signals==1&&beep_on&&led_on);
 assert(uses_line()&&!uses_heading());
 endpoint(1);assert(signals==2&&task2_leg==2&&uses_heading()&&!uses_line());
 assert(fabsf(wrap_degrees(yaw_reference-task2_start_heading-180))<.001f);
 endpoint(2);assert(signals==3&&task2_leg==3&&uses_line());
 endpoint(3);assert(signals==3&&!beep_on&&a==0&&b==0);
 for(int i=0;i<21;i++)step(10,0,0);
 assert(state==DONE&&signals==4&&completion_signal&&led_on&&beep_on);
 assert(start_ms==began&&task2_elapsed_ms==(uint32_t)(ms-began));
 assert(task2_elapsed_ms<30000&&rp[0]==0&&wp[0]==0&&!bluetooth_on);
 for(int i=0;i<50;i++)step(10,0,0);
 assert(state==DONE&&!completion_signal&&signals==4);
 for(unsigned i=0;i<4;i++)assert(strlen(oled_lines[i])<=21);
 assert(strcmp(oled_lines[0],"TASK 2 ONE LAP")==0&&strstr(oled_lines[2],"DONE DA T"));
 /* Repeated same-route request must not reset distance/timing while running. */
 task_start();uint32_t saved=start_ms;cmd("TASK2!");assert(start_ms==saved&&task2_leg==0);
 /* Modes cycle 1 -> half -> 2 -> 1; reboot is still task 1. */
 setup();hold();assert(key_motion==MOVE_LINE);hold();assert(key_motion==MOVE_TASK2);
 for(int i=0;i<200;i++)step(10,0,0);assert(key_motion==MOVE_TASK2&&yaw_phase==Y_NONE);
 hold();assert(key_motion==MOVE_ROUTE);setup();assert(key_motion==MOVE_ROUTE);
 /* Half-circle user's current tuned parameters and independent auto-stop survive. */
 setup();line_inputs=9;cmd("LF20!");calibrate();enable();press_key();
 assert(motion==MOVE_LINE&&state==RUNNING&&distance_goal==DPT_LINE_MM);
 assert(distance_cruise==200&&run_duration==DPT_LINE_TIMEOUT_MS);
 line_inputs=11;step(10,0,0);assert(targets[0]<targets[1]);line_inputs=13;step(10,0,0);assert(targets[0]>targets[1]);
 line_inputs=9;for(int i=0;i<1500&&state==RUNNING;i++)step(10,600,600);
 assert(state==BRAKING);for(int i=0;i<21;i++)step(10,0,0);assert(state==DONE&&signals==1);
 /* Early black on a white straight cannot prematurely declare B. */
 task_start();line_inputs=9;for(int i=0;i<50;i++)step(10,600,600);
 assert(task2_leg==0&&signals==0&&state==RUNNING);
 /* Missing arc entrance cannot keep driving indefinitely or falsely beep. */
 task_start();line_inputs=15;
 while(state==RUNNING)step(10,600,600);
 assert(state==LINE_LOST&&task2_leg==0&&signals==0&&a==0&&b==0);
 /* Mid-arc white/all-black/split are faults, not C/DA endpoints. */
 const unsigned bad[]={15,0,6};
 for(unsigned k=0;k<3;k++){
  fast_to_arc();line_inputs=bad[k];step(10,0,0);assert(a==0&&b==0);
  for(int i=0;i<10;i++)step(10,0,0);
  assert(state==(k?LINE_BAD:LINE_LOST)&&task2_leg==1&&signals==1&&!completion_signal);
  line_inputs=9;for(int i=0;i<30;i++)step(10,0,0);assert(state!=RUNNING&&a==0&&b==0);
 }
 /* Near-end white only succeeds with the expected 180-degree heading. */
 near_arc_end();yaw_angle=task2_start_heading;line_inputs=15;
 for(int i=0;i<11;i++)step(10,0,0);assert(state==LINE_LOST&&signals==1&&task2_leg==1);
 near_arc_end();line_inputs=15;step(10,0,0);
 assert(task2_edge_seen&&state==RUNNING&&a==0&&b==0);
 for(int i=0;i<3;i++)step(10,0,0);assert(task2_leg==2&&state==RUNNING&&signals==2);
 /* Exit bounce resets the stable-white timer, not an endpoint. */
 near_arc_end();line_inputs=15;step(10,0,0);step(10,0,0);line_inputs=9;step(10,0,0);
 assert(task2_leg==1&&!task2_edge_seen&&signals==1);
 /* Never drive beyond a missing exit line: bad geometry is bounded. */
 fast_to_arc();line_inputs=9;distance_mm[0]=distance_mm[1]=task2_leg_start+DPT_TASK2_ARC_MM+DPT_TASK2_EXIT_WINDOW_MM+1;
 step(10,0,0);assert(state==LINE_BAD&&a==0&&b==0&&signals==1);
 /* STOP/OFF/RX faults beat same-poll edge transitions and motor commits. */
 for(unsigned leg=0;leg<4;leg++)for(int fault=0;fault<6;fault++){
  task_start();for(unsigned n=0;n<leg;n++)endpoint(n);
  release_key();int sig=signals;
  if(fault==0){off=1;DualPITest_Poll(tick);assert(state==OFF);}
  if(fault==1){press_key();assert(state==STOPPED);}
  if(fault==2){cmd("STOP!TASK2!");assert(state==STOPPED);}
  if(fault==3){faults[0]=1;step(10,0,0);assert(state==UART_BAD);}
  if(fault==4){imu_fail=1;step(10,0,0);assert(state==YAW_BAD);}
  if(fault==5){off_during_read=1;step(10,0,0);assert(state==OFF);}
  assert(a==0&&b==0&&signals==sig&&!completion_signal&&!distance_tracking);
 }
 /* Both serial-port STOP/RX faults outrank simultaneous task2 selection/start. */
 for(unsigned port=0;port<2;port++)for(int fault=0;fault<2;fault++){
  task_start();assert(DualPITest_EnableBluetooth(&uart_mock)==HAL_ERROR);
  cmd("STOP!");assert(DualPITest_EnableBluetooth(&uart_mock)==HAL_OK);
  if(fault)faults[port]=1;else queue(port,"STOP!");queue(1-port,"TASK2!");
  DualPITest_Poll(tick);assert(state==(fault?UART_BAD:STOPPED)&&a==0&&b==0);
 }
 /* Full TX only drops telemetry and cannot change task2 sequencing. */
 fast_to_arc();full_tx=1;line_inputs=9;step(10,600,600);assert(state==RUNNING&&task2_leg==1);
 /* A new key restarts from A state after a fault, not from its interrupted leg. */
 fast_to_arc();line_inputs=15;for(int i=0;i<11;i++)step(10,0,0);assert(state==LINE_LOST);
 release_key();press_key();assert(state==RUNNING&&task2_leg==0&&distance_mm[0]==0);
 /* Heading guard applies on straights, is disabled during intended arcs. */
 task_start();yaw_angle=45;step(10,0,0);assert(state==YAW_BAD);
 fast_to_arc();yaw_angle=45;step(10,600,600);assert(state==RUNNING);
 /* Timeout/FAST/GAP/LOW/ADC/STALL/REV still latch; no false finish. */
 for(int fault=0;fault<7;fault++){
  task_start();
  if(fault==0){start_ms=ms-DPT_TASK2_TIMEOUT_MS;DualPITest_Poll(tick);assert(state==TIMEOUT);}
  if(fault==1){step(10,1100,100);assert(state==FAST);}
  if(fault==2){step(20,100,100);assert(state==GAP);}
  if(fault==3){bat=1100;for(int i=0;i<10;i++)step(10,100,100);assert(state==LOW);}
  if(fault==4){adc_fail=1;for(int i=0;i<10;i++)step(10,100,100);assert(state==ADC_BAD);}
  if(fault==5){for(int i=0;i<60;i++)step(10,0,0);assert(state==STALL);}
  if(fault==6){for(int i=0;i<5;i++)step(10,-40,100);assert(state==REVERSE);}
  assert(a==0&&b==0&&signals==0&&!distance_tracking);
 }
 /* Startup cannot be queued without READY; held boot or across ON never starts. */
 setup();hold();hold();enable();press_key();assert(state!=RUNNING&&key_action==KEY_WAIT);
 setup_key(1);for(int i=0;i<300;i++)step(10,0,0);assert(key_motion==MOVE_ROUTE&&key_events==0);
 task_start();release_key();press_key();assert(state==STOPPED);off=1;release_key();press_key();off=0;
 for(int i=0;i<200;i++)step(10,0,0);assert(state!=RUNNING);
 /* Running OLED, signal duration and elapsed calculation survive wrap. */
 task_start();start_ms=ms=0xfffffff0U;old_ms=adc_ms=log_ms=yaw_log_ms=key_log_ms=oled_log_ms=oled_render_ms=line_log_ms=ms;
 for(int i=0;i<30;i++)step(10,600,600);assert(state==RUNNING&&task2_leg==0);
 assert(strlen(oled_lines[2])<=21);
 cmd("STOP!");assert(task2_elapsed_ms==300);
 assert(sizeof banner-1<=160);
 (void)head;
 puts("PASS: calibrated1256mm BC endpoint regression; Task2 GPIO-only AB->BC->CD->DA->A four-point signals and final settled stop; white straight/black arc switching gated by distance, debounce and end heading; preserved half-circle settings and task1 regressions; three-mode keys, no BLE required, no premature entry/false arc loss endpoint; bounded missing edges, STOP/OFF/RX/IMU/FAST/GAP/LOW/ADC/STALL/REV/timeout protections, restart/held-key suppression, OLED limits and time wrap. Simulation is not field acceptance.");
}
