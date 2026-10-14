
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

static uint64_t trace_hash = UINT64_C(14695981039346656037);
static uint64_t trace_events;
static void trace_bytes(const void *data, size_t size) {
 const unsigned char *bytes=data;
 for(size_t i=0;i<size;i++){trace_hash^=bytes[i];trace_hash*=UINT64_C(1099511628211);}
}
static void trace_event(unsigned char kind) {
 ++trace_events;trace_bytes(&kind,sizeof kind);trace_bytes(&ms,sizeof ms);
 trace_bytes(&mock_dwt.CYCCNT,sizeof mock_dwt.CYCCNT);
}

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
 trace_event('g');unsigned port=p==GPIOC?1U:2U;trace_bytes(&port,sizeof port);trace_bytes(&pin,sizeof pin);trace_bytes(&value,sizeof value);
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
 trace_event('u');trace_bytes(&c,sizeof c);trace_bytes(&n,sizeof n);trace_bytes(s,n);(void)c;if(full_tx)return 0;assert(strlen(logs)+n<sizeof logs);strncat(logs,s,n);return 1;
}
int LabUART_TryWrite(const char*s,uint16_t n){return LabUART_TryWritePort(LAB_UART_USB,s,n);}
int LabUART_TryReadPort(LabUART_Port c,uint8_t*p){if(rp[c]==wp[c])return 0;*p=(uint8_t)rx[c][rp[c]++];return 1;}
int LabUART_TakeRxFaultPort(LabUART_Port c){int n=faults[c];faults[c]=0;return n;}
uint32_t LabUART_RxErrors(void){return 0;}uint32_t LabUART_TxDropped(void){return full_tx?1:0;}
uint32_t LabUART_RxErrorsPort(LabUART_Port c){(void)c;return 0;}
uint32_t LabUART_TxDroppedPort(LabUART_Port c){(void)c;return full_tx?1:0;}
static void motor(int aa,int bb){assert(aa>=-3000&&aa<=3000&&bb>=-3000&&bb<=3000);trace_event('m');trace_bytes(&aa,sizeof aa);trace_bytes(&bb,sizeof bb);a=aa;b=bb;}
void TaskOLED_Begin(uint32_t n){assert(n==72000000U);}
int TaskOLED_SetLines(const char *const text[4]){
 trace_event('o');for(unsigned i=0;i<4;i++)trace_bytes(text[i],strlen(text[i])+1U);
 for(unsigned i=0;i<4;++i){assert(strlen(text[i])<sizeof oled_lines[i]);strcpy(oled_lines[i],text[i]);}return 1;
}
void TaskOLED_Poll(uint32_t n){(void)n;}
uint32_t TaskOLED_LastIOUs(void){return 100U;}
#include "car_internal.h"
static void trace_poll(uint32_t n) {
 DualPITest_Poll(n);
 trace_event('p');
 trace_bytes(&CarApp_TestContext()->heading.phase,sizeof CarApp_TestContext()->heading.phase);
 trace_bytes(&CarApp_TestContext()->heading.available,sizeof CarApp_TestContext()->heading.available);
 trace_bytes(&CarApp_TestContext()->heading.cal_requested,sizeof CarApp_TestContext()->heading.cal_requested);
 trace_bytes(&CarApp_TestContext()->heading.warm_ms,sizeof CarApp_TestContext()->heading.warm_ms);
 trace_bytes(&CarApp_TestContext()->heading.cycles,sizeof CarApp_TestContext()->heading.cycles);
 trace_bytes(&CarApp_TestContext()->heading.samples,sizeof CarApp_TestContext()->heading.samples);
 trace_bytes(&CarApp_TestContext()->heading.io_us,sizeof CarApp_TestContext()->heading.io_us);
 trace_bytes(&CarApp_TestContext()->heading.sum,sizeof CarApp_TestContext()->heading.sum);
 trace_bytes(&CarApp_TestContext()->heading.min,sizeof CarApp_TestContext()->heading.min);
 trace_bytes(&CarApp_TestContext()->heading.max,sizeof CarApp_TestContext()->heading.max);
 trace_bytes(&CarApp_TestContext()->heading.bias,sizeof CarApp_TestContext()->heading.bias);
 trace_bytes(&CarApp_TestContext()->heading.angle,sizeof CarApp_TestContext()->heading.angle);
 trace_bytes(&CarApp_TestContext()->heading.previous_rate,sizeof CarApp_TestContext()->heading.previous_rate);
 trace_bytes(&CarApp_TestContext()->heading.reference,sizeof CarApp_TestContext()->heading.reference);
 trace_bytes(&CarApp_TestContext()->heading.error,sizeof CarApp_TestContext()->heading.error);
 trace_bytes(&CarApp_TestContext()->heading.diff,sizeof CarApp_TestContext()->heading.diff);
 trace_bytes(&CarApp_TestContext()->button.action,sizeof CarApp_TestContext()->button.action);
 trace_bytes(&CarApp_TestContext()->button.raw,sizeof CarApp_TestContext()->button.raw);
 trace_bytes(&CarApp_TestContext()->button.stable,sizeof CarApp_TestContext()->button.stable);
 trace_bytes(&CarApp_TestContext()->button.armed,sizeof CarApp_TestContext()->button.armed);
 trace_bytes(&CarApp_TestContext()->button.press_enabled,sizeof CarApp_TestContext()->button.press_enabled);
 trace_bytes(&CarApp_TestContext()->button.off_pending,sizeof CarApp_TestContext()->button.off_pending);
 trace_bytes(&CarApp_TestContext()->button.press_busy,sizeof CarApp_TestContext()->button.press_busy);
 trace_bytes(&CarApp_TestContext()->button.change_ms,sizeof CarApp_TestContext()->button.change_ms);
 trace_bytes(&CarApp_TestContext()->button.events,sizeof CarApp_TestContext()->button.events);
 trace_bytes(&CarApp_TestContext()->button.hold_ms,sizeof CarApp_TestContext()->button.hold_ms);
 trace_bytes(&CarApp_TestContext()->button.selected_motion,sizeof CarApp_TestContext()->button.selected_motion);
 trace_bytes(&CarApp_TestContext()->line.raw,sizeof CarApp_TestContext()->line.raw);
 trace_bytes(&CarApp_TestContext()->line.black,sizeof CarApp_TestContext()->line.black);
 trace_bytes(&CarApp_TestContext()->line.valid,sizeof CarApp_TestContext()->line.valid);
 trace_bytes(&CarApp_TestContext()->line.count,sizeof CarApp_TestContext()->line.count);
 trace_bytes(&CarApp_TestContext()->line.center_seen,sizeof CarApp_TestContext()->line.center_seen);
 trace_bytes(&CarApp_TestContext()->line.center_ready,sizeof CarApp_TestContext()->line.center_ready);
 trace_bytes(&CarApp_TestContext()->line.invalid_seen,sizeof CarApp_TestContext()->line.invalid_seen);
 trace_bytes(&CarApp_TestContext()->line.center_ms,sizeof CarApp_TestContext()->line.center_ms);
 trace_bytes(&CarApp_TestContext()->line.sample_ms,sizeof CarApp_TestContext()->line.sample_ms);
 trace_bytes(&CarApp_TestContext()->line.invalid_ms,sizeof CarApp_TestContext()->line.invalid_ms);
 trace_bytes(&CarApp_TestContext()->line.error,sizeof CarApp_TestContext()->line.error);
 trace_bytes(&CarApp_TestContext()->line.diff,sizeof CarApp_TestContext()->line.diff);
 trace_bytes(&CarApp_TestContext()->route.motion,sizeof CarApp_TestContext()->route.motion);
 trace_bytes(&CarApp_TestContext()->route.distance_mm,sizeof CarApp_TestContext()->route.distance_mm);
 trace_bytes(&CarApp_TestContext()->route.base,sizeof CarApp_TestContext()->route.base);
 trace_bytes(&CarApp_TestContext()->route.goal,sizeof CarApp_TestContext()->route.goal);
 trace_bytes(&CarApp_TestContext()->route.cruise,sizeof CarApp_TestContext()->route.cruise);
 trace_bytes(&CarApp_TestContext()->route.slow,sizeof CarApp_TestContext()->route.slow);
 trace_bytes(&CarApp_TestContext()->route.distance_count,sizeof CarApp_TestContext()->route.distance_count);
 trace_bytes(&CarApp_TestContext()->route.tracking,sizeof CarApp_TestContext()->route.tracking);
 trace_bytes(&CarApp_TestContext()->route.settle_seen,sizeof CarApp_TestContext()->route.settle_seen);
 trace_bytes(&CarApp_TestContext()->route.brake_ms,sizeof CarApp_TestContext()->route.brake_ms);
 trace_bytes(&CarApp_TestContext()->route.settle_ms,sizeof CarApp_TestContext()->route.settle_ms);
 trace_bytes(&CarApp_TestContext()->route.manual_duration,sizeof CarApp_TestContext()->route.manual_duration);
 trace_bytes(&CarApp_TestContext()->route.run_duration,sizeof CarApp_TestContext()->route.run_duration);
 trace_bytes(&CarApp_TestContext()->route.switch_ms,sizeof CarApp_TestContext()->route.switch_ms);
 trace_bytes(&CarApp_TestContext()->route.start_ms,sizeof CarApp_TestContext()->route.start_ms);
 trace_bytes(&CarApp_TestContext()->route.manual_speed,sizeof CarApp_TestContext()->route.manual_speed);
 trace_bytes(&CarApp_TestContext()->route.switching,sizeof CarApp_TestContext()->route.switching);
 trace_bytes(&CarApp_TestContext()->route.targets,sizeof CarApp_TestContext()->route.targets);
 trace_bytes(&CarApp_TestContext()->route.leg,sizeof CarApp_TestContext()->route.leg);
 trace_bytes(&CarApp_TestContext()->route.edge_seen,sizeof CarApp_TestContext()->route.edge_seen);
 trace_bytes(&CarApp_TestContext()->route.edge_ms,sizeof CarApp_TestContext()->route.edge_ms);
 trace_bytes(&CarApp_TestContext()->route.elapsed_ms,sizeof CarApp_TestContext()->route.elapsed_ms);
 trace_bytes(&CarApp_TestContext()->route.leg_start,sizeof CarApp_TestContext()->route.leg_start);
 trace_bytes(&CarApp_TestContext()->route.start_heading,sizeof CarApp_TestContext()->route.start_heading);
 trace_bytes(&CarApp_TestContext()->power.valid,sizeof CarApp_TestContext()->power.valid);
 trace_bytes(&CarApp_TestContext()->power.sample_ms,sizeof CarApp_TestContext()->power.sample_ms);
 trace_bytes(&CarApp_TestContext()->power.raw,sizeof CarApp_TestContext()->power.raw);
 trace_bytes(&CarApp_TestContext()->signal.active,sizeof CarApp_TestContext()->signal.active);
 trace_bytes(&CarApp_TestContext()->signal.ready_led_on,sizeof CarApp_TestContext()->signal.ready_led_on);
 trace_bytes(&CarApp_TestContext()->signal.ms,sizeof CarApp_TestContext()->signal.ms);
 trace_bytes(&CarApp_TestContext()->ui.render_ms,sizeof CarApp_TestContext()->ui.render_ms);
 trace_bytes(&CarApp_TestContext()->ui.oled_log_ms,sizeof CarApp_TestContext()->ui.oled_log_ms);
 trace_bytes(&CarApp_TestContext()->ui.line_log_ms,sizeof CarApp_TestContext()->ui.line_log_ms);
 trace_bytes(&CarApp_TestContext()->ui.heading_log_ms,sizeof CarApp_TestContext()->ui.heading_log_ms);
 trace_bytes(&CarApp_TestContext()->ui.button_log_ms,sizeof CarApp_TestContext()->ui.button_log_ms);
 trace_bytes(&CarApp_TestContext()->ui.motion_log_ms,sizeof CarApp_TestContext()->ui.motion_log_ms);
 trace_bytes(&CarApp_TestContext()->runtime.state,sizeof CarApp_TestContext()->runtime.state);
 trace_bytes(&CarApp_TestContext()->runtime.active,sizeof CarApp_TestContext()->runtime.active);
 trace_bytes(&CarApp_TestContext()->runtime.primed,sizeof CarApp_TestContext()->runtime.primed);
 trace_bytes(&CarApp_TestContext()->runtime.on_seen,sizeof CarApp_TestContext()->runtime.on_seen);
 trace_bytes(&CarApp_TestContext()->runtime.bluetooth_on,sizeof CarApp_TestContext()->runtime.bluetooth_on);
 trace_bytes(&CarApp_TestContext()->runtime.hz,sizeof CarApp_TestContext()->runtime.hz);
 trace_bytes(&CarApp_TestContext()->runtime.old_cycles,sizeof CarApp_TestContext()->runtime.old_cycles);
 trace_bytes(&CarApp_TestContext()->runtime.old_tick,sizeof CarApp_TestContext()->runtime.old_tick);
 trace_bytes(&CarApp_TestContext()->runtime.old_ms,sizeof CarApp_TestContext()->runtime.old_ms);
 trace_bytes(&CarApp_TestContext()->runtime.on_ms,sizeof CarApp_TestContext()->runtime.on_ms);
 trace_bytes(&CarApp_TestContext()->runtime.dt_us,sizeof CarApp_TestContext()->runtime.dt_us);
 trace_bytes(&CarApp_TestContext()->runtime.fault_wheel,sizeof CarApp_TestContext()->runtime.fault_wheel);
 for(unsigned i=0;i<2;i++)trace_bytes(&CarApp_TestContext()->wheel.items[i].old_count,sizeof CarApp_TestContext()->wheel.items[i].old_count);
 for(unsigned i=0;i<2;i++)trace_bytes(&CarApp_TestContext()->wheel.items[i].velocity,sizeof CarApp_TestContext()->wheel.items[i].velocity);
 for(unsigned i=0;i<2;i++)trace_bytes(&CarApp_TestContext()->wheel.items[i].integral,sizeof CarApp_TestContext()->wheel.items[i].integral);
 for(unsigned i=0;i<2;i++)trace_bytes(&CarApp_TestContext()->wheel.items[i].command,sizeof CarApp_TestContext()->wheel.items[i].command);
 for(unsigned i=0;i<2;i++)trace_bytes(&CarApp_TestContext()->wheel.items[i].bad_sign_us,sizeof CarApp_TestContext()->wheel.items[i].bad_sign_us);
 for(unsigned i=0;i<2;i++)trace_bytes(&CarApp_TestContext()->wheel.items[i].stall_us,sizeof CarApp_TestContext()->wheel.items[i].stall_us);
 for(unsigned i=0;i<LAB_UART_COUNT;i++){trace_bytes(CarApp_TestContext()->command.inputs[i].text,sizeof CarApp_TestContext()->command.inputs[i].text);trace_bytes(&CarApp_TestContext()->command.inputs[i].length,sizeof CarApp_TestContext()->command.inputs[i].length);trace_bytes(&CarApp_TestContext()->command.inputs[i].discard,sizeof CarApp_TestContext()->command.inputs[i].discard);trace_bytes(&CarApp_TestContext()->command.inputs[i].ms,sizeof CarApp_TestContext()->command.inputs[i].ms);}
}

static void setup_key(int held){
 ms=tick=0;memset(&mock_dwt,0,sizeof mock_dwt);memset(&mock_core,0,sizeof mock_core);
 l.count=r.count=0;memset(rx,0,sizeof rx);memset(rp,0,sizeof rp);memset(wp,0,sizeof wp);memset(faults,0,sizeof faults);
 logs[0]=0;signals=led_on=led1=led2=beep_on=0;key_input=held;off=1;adc_fail=imu_fail=off_during_read=full_tx=0;io_us=3000;bat=1320;gx=gy=0;gz=-424;
 assert(DualPITest_Begin(&uart_mock,&l,&r,&adc_mock,motor)==HAL_OK);
 assert(DualPITest_EnableBluetooth(&uart_mock)==HAL_OK);DualPITest_EnableHeading(1);
 trace_poll(tick);
}
static void setup(void){setup_key(0);}
static void step(unsigned n,int dl,int dr){
 /* Each call is scheduled from the preceding sample start, not I2C completion. */
 ms+=n;DWT->CYCCNT=CarApp_TestContext()->runtime.old_cycles+n*72000U;tick+=n/10U;
 l.count=(l.count+dl)&65535U;r.count=(r.count-dr)&65535U;
 trace_poll(tick);
}
static void queue(unsigned c,const char*s){size_t n=strlen(s);assert(wp[c]+n<sizeof rx[c]);memcpy(rx[c]+wp[c],s,n);wp[c]+=(unsigned)n;}
static void cmd(const char*s){queue(0,s);trace_poll(tick);}
static void enable(void){off=0;for(int i=0;i<20;i++)step(10,0,0);}
static void calibrate(void){cmd("CAL!");for(int i=0;i<705;i++)step(10,0,0);assert(CarApp_TestContext()->heading.phase==Y_READY&&CarApp_TestContext()->heading.samples==500&&fabsf(CarApp_TestContext()->heading.bias+424)<.001f&&a==0&&b==0);}
static void head(void){enable();cmd("GO!");step(10,0,0);assert(CarApp_TestContext()->runtime.state==RUNNING&&CarApp_TestContext()->route.motion==MOVE_HEADING&&a==1800&&b==-1800);}
static void release_key(void){key_input=0;for(int i=0;i<4;i++)step(10,0,0);}
static void press_key(void){key_input=1;for(int i=0;i<4;i++)step(10,0,0);}
int main(void){
 setup();enable();cmd("GO!");step(10,0,0);assert(a==0&&b==0&&CarApp_TestContext()->runtime.state!=RUNNING);cmd("CAL!");assert(CarApp_TestContext()->heading.phase==Y_NONE);
 setup();cmd("CAL!GO!");for(int i=0;i<705;i++)step(10,0,0);assert(CarApp_TestContext()->heading.phase==Y_READY&&CarApp_TestContext()->runtime.state!=RUNNING);enable();for(int i=0;i<5;i++)step(10,0,0);assert(a==0&&b==0);
 setup();calibrate();head();gz=886;for(int i=0;i<10;i++)step(10,150,150);assert(CarApp_TestContext()->heading.error<0&&CarApp_TestContext()->route.targets[LEFT]>60&&CarApp_TestContext()->route.targets[RIGHT]<60);cmd("STOP!");assert(CarApp_TestContext()->runtime.state==STOPPED&&a==0&&b==0&&CarApp_TestContext()->wheel.items[0].integral==0&&CarApp_TestContext()->wheel.items[1].integral==0);
 setup();calibrate();head();gz=-1734;for(int i=0;i<10;i++)step(10,150,150);assert(CarApp_TestContext()->heading.error>0&&CarApp_TestContext()->route.targets[LEFT]<60&&CarApp_TestContext()->route.targets[RIGHT]>60);
 setup();calibrate();head();gz=25776;for(int i=0;i<8;i++)step(10,150,150);assert(CarApp_TestContext()->runtime.state==RUNNING&&fabsf(CarApp_TestContext()->heading.diff+15)<.01f&&CarApp_TestContext()->route.targets[LEFT]==75&&CarApp_TestContext()->route.targets[RIGHT]==45);for(int i=0;i<5;i++)step(10,150,150);assert(CarApp_TestContext()->runtime.state==YAW_BAD&&a==0&&b==0);
 setup();calibrate();head();imu_fail=1;step(10,150,150);assert(CarApp_TestContext()->runtime.state==YAW_BAD&&a==0&&b==0);
 setup();calibrate();head();io_us=6000;step(10,150,150);assert(CarApp_TestContext()->runtime.state==YAW_BAD&&a==0&&b==0);
 setup();calibrate();head();gz=32760;step(10,150,150);assert(CarApp_TestContext()->runtime.state==YAW_BAD&&a==0&&b==0);
 setup();calibrate();head();off_during_read=1;step(10,150,150);assert(CarApp_TestContext()->runtime.state==OFF&&a==0&&b==0);
 setup();calibrate();head();for(int i=0;i<150;i++)step(10,150,150);cmd("GO!");for(int i=0;i<149;i++)step(10,150,150);assert(CarApp_TestContext()->runtime.state==DONE&&a==0&&b==0);
 setup();calibrate();head();queue(0,"GO!");queue(1,"STOP!");trace_poll(tick);assert(CarApp_TestContext()->runtime.state==STOPPED&&a==0&&b==0);
 setup();calibrate();head();queue(0,"STOP!");queue(1,"GO!");trace_poll(tick);assert(CarApp_TestContext()->runtime.state==STOPPED&&a==0&&b==0);
 setup();calibrate();head();faults[1]=1;trace_poll(tick);assert(CarApp_TestContext()->runtime.state==UART_BAD&&a==0&&b==0);
 setup();calibrate();head();step(20,150,150);assert(CarApp_TestContext()->runtime.state==GAP&&a==0&&b==0);
 setup();cmd("CAL!");for(int i=0;i<205;i++)step(10,0,0);step(10,1,0);assert(CarApp_TestContext()->heading.phase==Y_FAIL&&a==0&&b==0);
 setup();cmd("CAL!");for(int i=0;i<205;i++)step(10,0,0);off=0;step(10,0,0);assert(CarApp_TestContext()->heading.phase==Y_FAIL&&a==0&&b==0);
 setup();cmd("CAL!");for(int i=0;i<205;i++)step(10,0,0);gz=1000;step(10,0,0);gz=-424;for(int i=0;i<500;i++)step(10,0,0);assert(CarApp_TestContext()->heading.phase==Y_FAIL);
 setup();cmd("CAL!");cmd("STOP!");for(int i=0;i<705;i++)step(10,0,0);assert(CarApp_TestContext()->heading.phase==Y_NONE&&a==0&&b==0);
 setup();queue(0,"CAL!");queue(1,"STOP!");trace_poll(tick);assert(CarApp_TestContext()->heading.phase==Y_NONE);
 setup();calibrate();enable();cmd("F!");step(10,0,0);assert(a==3000&&b==-3000);for(int i=0;i<20;i++)step(10,300,300);cmd("GO!");assert(CarApp_TestContext()->route.motion==MOVE_FORWARD);cmd("STOP!");
 /* Existing commands work without successful MPU calibration. */
 setup();DualPITest_EnableHeading(0);enable();cmd("RUN!");step(10,0,0);assert(a==1800&&b==-1800);for(int i=0;i<299;i++)step(10,150,150);assert(CarApp_TestContext()->runtime.state==DONE);
 setup();enable();cmd("B!");step(10,0,0);assert(a==-3000&&b==3000);for(int i=0;i<20;i++)step(10,-300,-300);assert(CarApp_TestContext()->runtime.state==RUNNING);cmd("S!");assert(CarApp_TestContext()->runtime.state==STOPPED);
 setup();enable();cmd("L!");step(10,0,0);assert(CarApp_TestContext()->route.targets[LEFT]==60&&CarApp_TestContext()->route.targets[RIGHT]==120);cmd("R!");assert(a==0&&b==0);for(int i=0;i<20;i++)step(10,0,0);assert(CarApp_TestContext()->route.targets[LEFT]==120&&CarApp_TestContext()->route.targets[RIGHT]==60&&a>0&&b<0);cmd("STOP!");
 setup();enable();cmd("V150!");cmd("TIME3!");cmd("F!");step(10,0,0);assert(CarApp_TestContext()->route.targets[0]==150&&CarApp_TestContext()->route.run_duration==3000);cmd("V200!");assert(CarApp_TestContext()->route.manual_speed==150);cmd("STOP!");
 setup();calibrate();head();full_tx=1;for(int i=0;i<25;i++)step(10,150,150);assert(CarApp_TestContext()->runtime.state==RUNNING);cmd("STOP!");assert(a==0&&b==0);
 setup();calibrate();head();bat=1100;for(int i=0;i<10;i++)step(10,150,150);assert(CarApp_TestContext()->runtime.state==LOW&&a==0&&b==0);
 setup();calibrate();head();adc_fail=1;for(int i=0;i<10;i++)step(10,150,150);assert(CarApp_TestContext()->runtime.state==ADC_BAD&&a==0&&b==0);
 setup();calibrate();head();for(int i=0;i<60;i++)step(10,0,0);assert(CarApp_TestContext()->runtime.state==STALL&&a==0&&b==0);
 setup();calibrate();head();for(int i=0;i<5;i++)step(10,-40,150);assert(CarApp_TestContext()->runtime.state==REVERSE&&a==0&&b==0);
 setup();calibrate();head();step(10,1100,150);assert(CarApp_TestContext()->runtime.state==FAST&&a==0&&b==0);
 /* Calibration and reference survive DWT wrap; heading shortest-angle error. */
 assert(fabsf(CarMath_WrapDegrees(-358)-2)<.001f);assert(fabsf(CarMath_WrapDegrees(358)+2)<.001f);
 setup();calibrate();DWT->CYCCNT=CarApp_TestContext()->runtime.old_cycles=0xffff0000U;CarApp_TestContext()->heading.cycles=CarApp_TestContext()->runtime.old_cycles+io_us*72U;head();step(10,150,150);assert(CarApp_TestContext()->runtime.state==RUNNING);cmd("STOP!");

 /* Distance task uses encoder travel and settles before DONE. */
 setup();enable();cmd("GO20!");step(10,0,0);assert(CarApp_TestContext()->runtime.state!=RUNNING&&a==0&&b==0);
 setup();calibrate();enable();cmd("GO20!");step(10,0,0);
 assert(CarApp_TestContext()->runtime.state==RUNNING&&CarApp_TestContext()->route.motion==MOVE_DISTANCE&&CarApp_TestContext()->route.run_duration==10000&&CarApp_TestContext()->route.distance_mm[0]==0&&CarApp_TestContext()->route.distance_mm[1]==0);
 int slowed=0,floor_seen=0;
 for(int i=0;i<400&&CarApp_TestContext()->runtime.state==RUNNING;i++){
  step(10,176,176);
  if(CarApp_TestContext()->route.base<59.0f)slowed=1;
  if(CarApp_TestContext()->route.base==40.0f)floor_seen=1;
  assert(CarApp_TestContext()->route.base>=40.0f&&CarApp_TestContext()->route.base<=60.0f);
 }
 assert(CarApp_TestContext()->runtime.state==BRAKING&&slowed&&floor_seen&&a==0&&b==0);
 assert((CarApp_TestContext()->route.distance_mm[0]+CarApp_TestContext()->route.distance_mm[1])*.5f>=200.0f&&(CarApp_TestContext()->route.distance_mm[0]+CarApp_TestContext()->route.distance_mm[1])*.5f<200.7f);
 assert(CarApp_TestContext()->wheel.items[0].integral==0&&CarApp_TestContext()->wheel.items[1].integral==0);
 for(int i=0;i<5;i++)step(10,30,30); /* coast is included in final mileage */
 float coast=(CarApp_TestContext()->route.distance_mm[0]+CarApp_TestContext()->route.distance_mm[1])*.5f;
 for(int i=0;i<20;i++)step(10,0,0);assert(CarApp_TestContext()->runtime.state==BRAKING);
 step(10,0,0);assert(CarApp_TestContext()->runtime.state==DONE&&fabsf((CarApp_TestContext()->route.distance_mm[0]+CarApp_TestContext()->route.distance_mm[1])*.5f-coast)<.001f);
 float final_distance=CarApp_TestContext()->route.distance_mm[0];for(int i=0;i<30;i++)step(10,20,20);
 assert(fabsf(CarApp_TestContext()->route.distance_mm[0]-final_distance)<.001f&&strstr(logs,"M=DIST")&&strstr(logs," G=200"));
 cmd("GO20!");assert(CarApp_TestContext()->runtime.state==RUNNING&&CarApp_TestContext()->route.distance_mm[0]==0&&CarApp_TestContext()->route.distance_mm[1]==0);cmd("STOP!");assert(CarApp_TestContext()->runtime.state==STOPPED&&CarApp_TestContext()->route.tracking==0&&a==0&&b==0);
 /* Waiting for settle never accepts a movement command. */
 setup();calibrate();enable();cmd("GO20!");
 for(int i=0;i<400&&CarApp_TestContext()->runtime.state==RUNNING;i++)step(10,176,176);assert(CarApp_TestContext()->runtime.state==BRAKING);
 cmd("F!");cmd("GO20!");assert(CarApp_TestContext()->runtime.state==BRAKING&&a==0&&b==0);cmd("STOP!");assert(CarApp_TestContext()->runtime.state==STOPPED&&!CarApp_TestContext()->route.tracking);
 /* Braking timeout is a fault, not successful completion. */
 setup();calibrate();enable();cmd("GO20!");
 for(int i=0;i<400&&CarApp_TestContext()->runtime.state==RUNNING;i++)step(10,176,176);
 for(int i=0;i<150;i++)step(10,30,30);assert(CarApp_TestContext()->runtime.state==SETTLE_BAD&&a==0&&b==0&&!CarApp_TestContext()->route.tracking);
 /* 3 seconds and repeated commands don't finish or reset distance. */
 setup();calibrate();enable();cmd("GO20!");for(int i=0;i<305;i++)step(10,150,150);
 assert(CarApp_TestContext()->runtime.state==RUNNING&&(CarApp_TestContext()->route.distance_mm[0]+CarApp_TestContext()->route.distance_mm[1])*.5f<200);
 float before_distance=CarApp_TestContext()->route.distance_mm[0];uint32_t before_start=CarApp_TestContext()->route.start_ms;cmd("GO20!");
 assert(CarApp_TestContext()->route.distance_mm[0]==before_distance&&CarApp_TestContext()->route.start_ms==before_start);
 /* Slow progress reaches the fixed safety deadline. */
 setup();calibrate();enable();cmd("GO20!");for(int i=0;i<999;i++)step(10,20,20);assert(CarApp_TestContext()->runtime.state==RUNNING);
 cmd("GO20!");step(10,20,20);assert(CarApp_TestContext()->runtime.state==TIMEOUT&&a==0&&b==0&&!CarApp_TestContext()->route.tracking);
 /* Signed mean, separate CPRs, and pre-start motion exclusion. */
 setup();calibrate();enable();l.count+=100;r.count=(r.count-100)&65535;cmd("GO20!");step(10,0,0);
 assert(CarApp_TestContext()->route.distance_mm[0]==0&&CarApp_TestContext()->route.distance_mm[1]==0);step(10,176,150);
 assert(fabsf(CarApp_TestContext()->route.distance_mm[0]-176*DPT_CIRC_M*1000/DPT_LEFT_CPR)<.0001f);
 assert(fabsf(CarApp_TestContext()->route.distance_mm[1]-150*DPT_CIRC_M*1000/DPT_RIGHT_CPR)<.0001f);
 float forward_left=CarApp_TestContext()->route.distance_mm[0];step(10,-30,150);assert(CarApp_TestContext()->route.distance_mm[0]<forward_left);cmd("STOP!");
 /* Encoder counter rollover and time rollover retain the same distance task. */
 setup();calibrate();enable();l.count=65500;r.count=40;CarApp_TestContext()->wheel.items[0].old_count=65500;CarApp_TestContext()->wheel.items[1].old_count=40;
 cmd("GO20!");for(int i=0;i<400&&CarApp_TestContext()->runtime.state==RUNNING;i++)step(10,176,176);assert(CarApp_TestContext()->runtime.state==BRAKING);
 setup();calibrate();enable();ms=0xfffffff0U;CarApp_TestContext()->runtime.on_ms=ms-200U;CarApp_TestContext()->power.sample_ms=ms;CarApp_TestContext()->ui.motion_log_ms=CarApp_TestContext()->ui.heading_log_ms=ms;CarApp_TestContext()->runtime.old_ms=ms;
 cmd("GO20!");for(int i=0;i<400&&CarApp_TestContext()->runtime.state==RUNNING;i++)step(10,176,176);assert(CarApp_TestContext()->runtime.state==BRAKING);
 for(int i=0;i<21;i++)step(10,0,0);assert(CarApp_TestContext()->runtime.state==DONE);
 /* Existing guards and UART priority apply to distance motion and braking. */
 setup();calibrate();enable();queue(0,"GO20!");queue(1,"STOP!");trace_poll(tick);assert(CarApp_TestContext()->runtime.state==STOPPED);
 setup();calibrate();enable();cmd("GO20!");step(10,176,176);off=1;trace_poll(tick);assert(CarApp_TestContext()->runtime.state==OFF&&a==0&&b==0);
 setup();calibrate();enable();cmd("GO20!");step(10,1100,150);assert(CarApp_TestContext()->runtime.state==FAST&&a==0&&b==0&&!CarApp_TestContext()->route.tracking);
 setup();calibrate();enable();cmd("GO20!");step(20,176,176);assert(CarApp_TestContext()->runtime.state==GAP&&CarApp_TestContext()->route.distance_mm[0]==0);
 setup();calibrate();enable();cmd("GO20!");step(10,176,176);imu_fail=1;step(10,176,176);assert(CarApp_TestContext()->runtime.state==YAW_BAD&&a==0&&b==0);
 setup();calibrate();enable();cmd("GO20!");full_tx=1;for(int i=0;i<40;i++)step(10,176,176);cmd("STOP!");assert(CarApp_TestContext()->runtime.state==STOPPED&&a==0&&b==0);
 setup();calibrate();enable();cmd("GO20!");for(int i=0;i<400&&CarApp_TestContext()->runtime.state==RUNNING;i++)step(10,176,176);off=1;trace_poll(tick);assert(CarApp_TestContext()->runtime.state==OFF&&!CarApp_TestContext()->route.tracking);
 setup();calibrate();enable();cmd("GO20!");for(int i=0;i<400&&CarApp_TestContext()->runtime.state==RUNNING;i++)step(10,176,176);step(20,0,0);assert(CarApp_TestContext()->runtime.state==GAP&&!CarApp_TestContext()->route.tracking);
 setup();calibrate();enable();cmd("GO20!");for(int i=0;i<400&&CarApp_TestContext()->runtime.state==RUNNING;i++)step(10,176,176);faults[1]=1;trace_poll(tick);assert(CarApp_TestContext()->runtime.state==UART_BAD&&!CarApp_TestContext()->route.tracking);
 setup();calibrate();enable();cmd("GO20!");for(int i=0;i<400&&CarApp_TestContext()->runtime.state==RUNNING;i++)step(10,176,176);imu_fail=1;step(10,0,0);assert(CarApp_TestContext()->runtime.state==YAW_BAD&&!CarApp_TestContext()->route.tracking);
 setup();calibrate();enable();cmd("GO20!");for(int i=0;i<400&&CarApp_TestContext()->runtime.state==RUNNING;i++)step(10,176,176);
 step(10,0,0);step(10,0,0);step(10,30,30);for(int i=0;i<20;i++)step(10,0,0);assert(CarApp_TestContext()->runtime.state==BRAKING);step(10,0,0);assert(CarApp_TestContext()->runtime.state==DONE);

 /* 1m route keeps its goal, speed and deadline separate from GO20/manual configuration. */
 setup();enable();cmd("GO100!");assert(CarApp_TestContext()->runtime.state!=RUNNING&&!led_on&&!beep_on);
 setup();calibrate();enable();cmd("V200!TIME1!GO100!");
 assert(CarApp_TestContext()->runtime.state==RUNNING&&CarApp_TestContext()->route.motion==MOVE_ROUTE&&CarApp_TestContext()->route.goal==1000&&CarApp_TestContext()->route.cruise==120&&CarApp_TestContext()->route.slow==100&&CarApp_TestContext()->route.run_duration==13500);
 assert(CarApp_TestContext()->route.targets[0]==40&&CarApp_TestContext()->route.targets[1]==40&&!CarApp_TestContext()->signal.active&&!signals);
 step(10,120,120);assert(CarApp_TestContext()->route.base>40&&CarApp_TestContext()->route.base<=41.1f);
 for(int i=0;i<40;i++)step(10,300,300);
 assert(CarApp_TestContext()->route.base>80&&CarApp_TestContext()->route.base<83&&CarApp_TestContext()->route.distance_mm[0]>40);
 float travelled=CarApp_TestContext()->route.distance_mm[0];uint32_t started=CarApp_TestContext()->route.start_ms;
 cmd("GO20!GO100!");assert(CarApp_TestContext()->route.distance_mm[0]==travelled&&CarApp_TestContext()->route.goal==1000&&CarApp_TestContext()->route.start_ms==started);
 for(int i=0;i<40;i++)step(10,300,300);assert(CarApp_TestContext()->route.base==120);
 gz=886;step(10,300,300);assert(CarApp_TestContext()->route.targets[0]>CarApp_TestContext()->route.targets[1]);gz=-424;
 int slow_seen=0, floor_seen_1m=0;
 for(int i=0;i<1100&&CarApp_TestContext()->runtime.state==RUNNING;i++){
  step(10,300,300);
  if(CarApp_TestContext()->route.distance_mm[0]>905&&CarApp_TestContext()->route.base<115)slow_seen=1;
  if(CarApp_TestContext()->route.distance_mm[0]>970&&CarApp_TestContext()->route.base==40)floor_seen_1m=1;
  assert(CarApp_TestContext()->route.targets[0]>=0&&CarApp_TestContext()->route.targets[1]>=0&&fabsf(CarApp_TestContext()->heading.diff)<=15.01f);
 }
 assert(CarApp_TestContext()->runtime.state==BRAKING&&slow_seen&&floor_seen_1m&&a==0&&b==0&&!signals);
 for(int i=0;i<5;i++)step(10,30,30);assert(!signals);
 for(int i=0;i<21;i++)step(10,0,0);
 assert(CarApp_TestContext()->runtime.state==DONE&&signals==1&&CarApp_TestContext()->signal.active&&led_on&&beep_on&&a==0&&b==0);
 float stopped_at=CarApp_TestContext()->route.distance_mm[0];
 for(int i=0;i<49;i++)step(10,0,0);assert(CarApp_TestContext()->signal.active&&signals==1);
 step(10,0,0);assert(!CarApp_TestContext()->signal.active&&!led_on&&!beep_on&&signals==1&&CarApp_TestContext()->route.distance_mm[0]==stopped_at);
 assert(strstr(logs," G=1000 SIG=1")&&strstr(logs," G=1000 SIG=0"));
 for(int i=0;i<50;i++)step(10,0,0);assert(signals==1&&CarApp_TestContext()->runtime.state==DONE);
 cmd("GO20!");assert(CarApp_TestContext()->runtime.state==RUNNING&&CarApp_TestContext()->route.motion==MOVE_DISTANCE&&CarApp_TestContext()->route.goal==200&&CarApp_TestContext()->route.cruise==60&&CarApp_TestContext()->route.slow==40&&CarApp_TestContext()->route.run_duration==10000&&CarApp_TestContext()->route.targets[0]==60);
 cmd("STOP!");assert(!signals||signals==1);assert(!CarApp_TestContext()->signal.active);
 /* Partial and unknown commands cannot start a 1m movement. */
 setup();calibrate();enable();cmd("GO10!GO1000!GO0100!");assert(CarApp_TestContext()->runtime.state!=RUNNING);
 queue(0,"GO");queue(1,"100!");trace_poll(tick);assert(CarApp_TestContext()->runtime.state!=RUNNING);
 /* Deadline is a fault, never a completion signal. */
 setup();calibrate();enable();cmd("GO100!");
 for(int i=0;i<1349;i++)step(10,20,20);assert(CarApp_TestContext()->runtime.state==RUNNING);
 cmd("GO100!");step(10,20,20);assert(CarApp_TestContext()->runtime.state==TIMEOUT&&!signals&&!CarApp_TestContext()->signal.active&&a==0&&b==0);
 /* STOP on either input wins over GO100. */
 for(int port=0;port<2;port++){
  setup();calibrate();enable();queue((unsigned)port,"GO100!");queue((unsigned)(1-port),"STOP!");trace_poll(tick);
  assert(CarApp_TestContext()->runtime.state==STOPPED&&!signals&&a==0&&b==0);
 }
 /* Route safety gates apply in RUN and BRAKE, with no false DONE or sound/light. */
 for(int brake=0;brake<2;brake++)for(int fault=0;fault<7;fault++){
  setup();calibrate();enable();cmd("GO100!");
  if(brake){for(int i=0;i<1100&&CarApp_TestContext()->runtime.state==RUNNING;i++)step(10,300,300);assert(CarApp_TestContext()->runtime.state==BRAKING);}
  if(fault==0){off=1;trace_poll(tick);assert(CarApp_TestContext()->runtime.state==OFF);}
  if(fault==1){cmd("STOP!");assert(CarApp_TestContext()->runtime.state==STOPPED);}
  if(fault==2){faults[1]=1;trace_poll(tick);assert(CarApp_TestContext()->runtime.state==UART_BAD);}
  if(fault==3){imu_fail=1;step(10,0,0);assert(CarApp_TestContext()->runtime.state==YAW_BAD);}
  if(fault==4){step(20,0,0);assert(CarApp_TestContext()->runtime.state==GAP);}
  if(fault==5){bat=1100;for(int i=0;i<10;i++)step(10,20,20);assert(CarApp_TestContext()->runtime.state==LOW);}
  if(fault==6){adc_fail=1;for(int i=0;i<10;i++)step(10,20,20);assert(CarApp_TestContext()->runtime.state==ADC_BAD);}
  assert(!CarApp_TestContext()->signal.active&&!signals&&a==0&&b==0&&!CarApp_TestContext()->route.tracking);
 }
 setup();calibrate();enable();cmd("GO100!");
 for(int i=0;i<1100&&CarApp_TestContext()->runtime.state==RUNNING;i++)step(10,300,300);
 cmd("GO20!GO100!F!");assert(CarApp_TestContext()->runtime.state==BRAKING&&CarApp_TestContext()->route.motion==MOVE_ROUTE);
 for(int i=0;i<150;i++)step(10,30,30);assert(CarApp_TestContext()->runtime.state==SETTLE_BAD&&!signals);
 /* A stopped indication is cancelled promptly by STOP/OFF/recalibration or new motion. */
 for(int action=0;action<4;action++){
  setup();calibrate();enable();cmd("GO100!");
  for(int i=0;i<1100&&CarApp_TestContext()->runtime.state==RUNNING;i++)step(10,300,300);
  for(int i=0;i<21;i++)step(10,0,0);assert(CarApp_TestContext()->signal.active&&signals==1);
  if(action==0)cmd("STOP!");
  if(action==1){off=1;trace_poll(tick);}
  if(action==2)cmd("GO20!");
  if(action==3){off=1;cmd("CAL!");assert(CarApp_TestContext()->heading.phase==Y_WARM);}
  assert(!CarApp_TestContext()->signal.active&&!led_on&&!beep_on&&signals==1);
 }
 /* Timer wrap is valid for the 1m deadline and the once-only 500ms signal. */
 setup();calibrate();enable();ms=0xfffffff0U;CarApp_TestContext()->runtime.on_ms=ms-200U;CarApp_TestContext()->power.sample_ms=ms;CarApp_TestContext()->ui.motion_log_ms=CarApp_TestContext()->ui.heading_log_ms=ms;CarApp_TestContext()->runtime.old_ms=ms;
 cmd("GO100!");step(10,120,120);assert(CarApp_TestContext()->runtime.state==RUNNING&&CarApp_TestContext()->route.base<=41.1f);
 for(int i=0;i<1100&&CarApp_TestContext()->runtime.state==RUNNING;i++)step(10,300,300);assert(CarApp_TestContext()->runtime.state==BRAKING);
 for(int i=0;i<21;i++)step(10,0,0);assert(signals==1&&CarApp_TestContext()->signal.active);
 ms=0xfffffff0U;CarApp_TestContext()->signal.ms=ms;CarApp_TestContext()->runtime.old_ms=ms;CarApp_TestContext()->power.sample_ms=ms;CarApp_TestContext()->ui.motion_log_ms=CarApp_TestContext()->ui.heading_log_ms=ms;
 for(int i=0;i<50;i++)step(10,0,0);assert(!CarApp_TestContext()->signal.active&&signals==1);
 /* A blocked transmit queue cannot delay stopping or ending the indication. */
 setup();calibrate();enable();cmd("GO100!");
 for(int i=0;i<1100&&CarApp_TestContext()->runtime.state==RUNNING;i++)step(10,300,300);
 for(int i=0;i<21;i++)step(10,0,0);full_tx=1;
 for(int i=0;i<50;i++)step(10,0,0);assert(!CarApp_TestContext()->signal.active&&signals==1&&a==0&&b==0);


 setup();calibrate();enable();press_key();assert(CarApp_TestContext()->runtime.state==RUNNING&&CarApp_TestContext()->route.motion==MOVE_ROUTE);
 release_key();press_key();assert(CarApp_TestContext()->runtime.state==STOPPED&&a==0&&b==0);
 puts("PASS: prior serial GO100/GO20/heading/manual motion regression, timing/MPU/ADC/encoder/UART protections, braking/coast/settle/completion and unchanged ON key start/busy STOP.");
 printf("TRACE hash=%016llx events=%llu\n",(unsigned long long)trace_hash,(unsigned long long)trace_events);
}
