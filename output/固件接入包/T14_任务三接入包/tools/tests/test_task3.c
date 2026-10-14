/* Reuse the existing mock hardware, run the actual scheduler and all modules. */
#define main task2_regression_main
#include "test_task2.c"
#undef main

static CarAppContext *app(void) { return CarApp_TestContext(); }
static void task3_start(void)
{
 setup(); line_inputs=15;
 hold();hold();hold();assert(app()->button.selected_motion==MOVE_TASK3);
 release_key();press_key();release_key();
 for(unsigned i=0;i<705;i++)step(10,0,0);
 assert(app()->heading.phase==Y_READY); enable();press_key();
 assert(app()->runtime.state==RUNNING&&app()->route.motion==MOVE_TASK3);
 assert(!app()->runtime.bluetooth_on && wp[0]==0 && wp[1]==0);
 assert(app()->route.run_duration==40000U && signals==1);
}
static float join_target(void)
{
 CarTask3 *t=&app()->route.task3;
 return CarMath_WrapDegrees(t->start_heading_deg+t->diagonal_angle_deg+
    (t->leg==2U ? 180.0f+t->diagonal_angle_deg : (t->leg==3U ? 180.0f : 0.0f)));
}
static void step_route(void)
{
 CarTask3 *t=&app()->route.task3;
 float p=progress();int dl=0,dr=0;
 switch(t->phase) {
 case T3_DIAGONAL:
  app()->heading.angle=CarMath_WrapDegrees(t->start_heading_deg+(t->leg==2U?180+2*t->diagonal_angle_deg:0));
  line_inputs=p>=t->diagonal_mm?9:15;
  if(app()->route.targets[0]>0)dl=dr=610;
  break;
 case T3_ARC:
  line_inputs=p>=t->config.arc_mm?15:9;
  app()->heading.angle=CarMath_WrapDegrees(t->start_heading_deg+t->diagonal_angle_deg+
      (t->leg==1U ? fminf(p/t->config.arc_mm,1.0f)*180.0f : 180.0f-fminf(p/t->config.arc_mm,1.0f)*180.0f));
  if(app()->route.targets[0]>0)dl=dr=610;
  break;
 case T3_ALIGN: {
  float e=CarMath_WrapDegrees(join_target()-app()->heading.angle);
  float turn=fmaxf(-2.0f,fminf(2.0f,e));app()->heading.angle=CarMath_WrapDegrees(app()->heading.angle+turn);
  if(app()->route.targets[0]>0)dl=177;
  if(app()->route.targets[1]>0)dr=176;
  break;
 }
 case T3_CAPTURE: line_inputs=9;break;
 default: break;
 }
 step(10,dl,dr);
 assert(a>=0 && b<=0); /* Physical mapping, no backward wheel command. */
 if(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_ALIGN) {
  if(app()->route.targets[0]==0)assert(app()->wheel.items[0].command==0);
  if(app()->route.targets[1]==0)assert(app()->wheel.items[1].command==0);
 }
}
static void reach_phase(CarTask3Phase phase)
{
 for(unsigned i=0;i<2500&&app()->runtime.state==RUNNING&&app()->route.task3.phase!=phase;i++)step_route();
 assert(app()->runtime.state==RUNNING && app()->route.task3.phase==phase);
}
int main(void)
{
 task3_start();uint32_t start=app()->route.start_ms;
 cmd("TASK3!");assert(app()->route.start_ms==start);
 for(unsigned i=0;i<3500&&app()->runtime.state==RUNNING;i++)step_route();
 assert(app()->runtime.state==DONE && app()->route.task3.phase==T3_DONE && signals==5 && a==0 && b==0);
 assert(app()->route.elapsed_ms<40000 && app()->route.leg==3U && rp[0]==wp[0] && wp[1]==0);
 for(unsigned i=0;i<50;i++)step(10,0,0);
 assert(signals==5 && !app()->signal.active && strcmp(oled_lines[0],"TASK 3 AC CB BD DA")==0);
 assert(strstr(oled_lines[2],"DONE DA T"));for(unsigned i=0;i<4;i++)assert(strlen(oled_lines[i])<=21);
 /* Busy new button press cancels; held press cannot restart. */
 task3_start();release_key();press_key();assert(app()->runtime.state==STOPPED && a==0 && b==0 && app()->route.task3.phase==T3_FAULT);
 for(unsigned i=0;i<200;i++)step(10,0,0);assert(app()->runtime.state==STOPPED);
 release_key();press_key();assert(app()->runtime.state==RUNNING && app()->route.task3.leg==0);
 /* Turns bypass the legacy 20deg heading trip, zero wheel clears old integral. */
 task3_start();reach_phase(T3_ALIGN);app()->wheel.items[0].integral=1700;step(10,0,0);
 assert(app()->runtime.state==RUNNING && app()->route.targets[0]==0 && app()->wheel.items[0].integral==0 && b==0 && a>0);
 /* Preserve specific app protection reasons and cancel task core in each phase. */
 for(unsigned k=0;k<12;k++) {
  task3_start();if(k>=6)reach_phase(T3_ARC);
  switch(k%6) {
  case 0:off=1;step(10,0,0);assert(app()->runtime.state==OFF);break;
  case 1:queue(0,"STOP!TASK3!");step(10,0,0);assert(app()->runtime.state==STOPPED);break;
  case 2:imu_fail=1;step(10,0,0);assert(app()->runtime.state==YAW_BAD);break;
  case 3:adc_fail=1;app()->power.sample_ms=ms-100;step(10,0,0);assert(app()->runtime.state==ADC_BAD);break;
  case 4:faults[0]=1;step(10,0,0);assert(app()->runtime.state==UART_BAD);break;
  case 5:step(30,0,0);assert(app()->runtime.state==GAP);break;
  }
  assert(app()->route.task3.phase==T3_FAULT && a==0 && b==0);
 }
 task3_start();bat=1199;app()->power.sample_ms=ms-100;step(10,0,0);assert(app()->runtime.state==LOW && a==0 && b==0);
 task3_start();off_during_read=1;step(10,0,0);assert(app()->runtime.state==OFF && a==0 && b==0);
 task3_start();step(10,2000,0);assert(app()->runtime.state==FAST && a==0 && b==0);
 task3_start();for(unsigned i=0;i<5;i++)step(10,0,-100);assert(app()->runtime.state==REVERSE && a==0 && b==0);
 task3_start();for(unsigned i=0;i<55;i++)step(10,0,0);assert(app()->runtime.state==STALL && a==0 && b==0);
 task3_start();reach_phase(T3_ARC);line_inputs=15;
 for(unsigned i=0;i<11;i++)step(10,0,0);
 assert(app()->runtime.state==LINE_LOST && app()->route.task3.fault==T3_LOST && a==0 && b==0);
 task3_start();reach_phase(T3_ALIGN);app()->route.task3.phase_start_mm=-100;step(10,0,0);
 assert(app()->runtime.state==YAW_BAD && app()->route.task3.fault==T3_TURN && a==0 && b==0);
 task3_start();app()->route.start_ms=ms-40000U;step(10,0,0);
 assert(app()->runtime.state==TIMEOUT && app()->route.task3.fault==T3_TIMEOUT && a==0 && b==0);
 /* USB and BT fault/STOP priority; TX congestion doesn't stop the route. */
 task3_start();assert(DualPITest_EnableBluetooth(&uart_mock)==HAL_ERROR);
 task3_start();full_tx=1;for(unsigned i=0;i<100;i++)step_route();assert(app()->runtime.state==RUNNING);
 full_tx=0;cmd("STOP!");assert(app()->runtime.state==STOPPED);
 setup();line_inputs=15;hold();hold();hold();calibrate();assert(DualPITest_EnableBluetooth(&uart_mock)==HAL_OK);
 enable();press_key();queue(1,"STOP!");queue(0,"TASK3!");step(10,0,0);assert(app()->runtime.state==STOPPED&&a==0&&b==0);
 setup();line_inputs=15;hold();hold();hold();calibrate();assert(DualPITest_EnableBluetooth(&uart_mock)==HAL_OK);
 enable();press_key();faults[1]=1;step(10,0,0);assert(app()->runtime.state==UART_BAD&&a==0&&b==0);
 /* Millis and encoder counters cross their wrap boundaries during a full lap. */
 setup();line_inputs=15;hold();hold();hold();calibrate();
 ms=UINT32_MAX-15000U;app()->runtime.old_ms=ms;app()->line.sample_ms=ms;app()->power.sample_ms=ms;
 enable();press_key();for(unsigned i=0;i<3500&&app()->runtime.state==RUNNING;i++)step_route();
 assert(app()->runtime.state==DONE && signals==5 && app()->route.elapsed_ms<40000U);
 puts("PASS: actual CarApp scheduler Task3 GPIO selection/start, AC->CB->BD->DA settled completion, five signals, zero inner PI, no reverse commands, old heading gate isolation, STOP/OFF/RX/MPU/ADC/LOW/FAST/REV/STALL/GAP, line/turn/time guards, restart, telemetry congestion, encoder/millis wrap. Simulation only.");
 return 0;
}
