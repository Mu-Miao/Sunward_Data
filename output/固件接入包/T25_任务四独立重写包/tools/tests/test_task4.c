/* Task4 tests run actual modules/scheduler with mock HAL, never touch hardware. */
#define CAR_TASK3_TEST_MAIN task3_regression_main
#include "test_task3.c"
#undef CAR_TASK3_TEST_MAIN

static void task4_select(void)
{
 setup();line_inputs=15;hold();hold();hold();hold();
 assert(app()->button.selected_motion==MOVE_TASK4 && app()->runtime.state!=RUNNING);
 assert(app()->route.goal>20280 && app()->route.goal<20300);
 assert(strcmp(oled_lines[0],"TASK 4 REBUILD V1")==0);
}
static void task4_start(void)
{
 task4_select();release_key();press_key();release_key();
 for(unsigned i=0;i<705;i++)step(10,0,0);
 assert(app()->heading.phase==Y_READY);enable();press_key();
 assert(app()->runtime.state==RUNNING && app()->route.motion==MOVE_TASK4);
 assert(app()->route.task3.config.deadline_ms==80000U && app()->route.run_duration==160000U);
 assert(app()->route.task4.completed_laps==0 && app()->route.task3.phase==T3_START_ALIGN && signals==1);
 assert(!app()->runtime.bluetooth_on && wp[0]==0 && wp[1]==0);
}
static uint32_t target_follow_four(int prior_profile)
{
 task4_start();
 if(prior_profile) {
  app()->route.task3.config.diagonal_mmps=200;
  app()->route.task3.config.turn_mmps=60;
  app()->route.task3.config.acceleration_mmps2=100;
  app()->route.task3.config.arc_acceleration_mmps2=100;
  app()->route.task3.config.approach_mm=150;
  app()->route.task3.config.minimum_mmps=40;
 }
 float residue[2]={0,0};
 for(unsigned k=0;k<16500 && app()->runtime.state==RUNNING;k++) {
  CarTask3 *t=&app()->route.task3;float p=progress();
  if(t->phase==T3_DIAGONAL) {
   app()->heading.angle=CarMath_WrapDegrees(t->start_heading_deg+(t->leg==2U?180+2*t->diagonal_angle_deg:0));
   line_inputs=p>=t->diagonal_mm?9:15;
  } else if(t->phase==T3_ARC) {
   line_inputs=p>=t->config.arc_mm?15:9;
   app()->heading.angle=CarMath_WrapDegrees(t->start_heading_deg+t->diagonal_angle_deg+
     (t->leg==1U?fminf(p/t->config.arc_mm,1)*180:180-fminf(p/t->config.arc_mm,1)*180));
  } else if(t->phase==T3_CAPTURE) line_inputs=9;
  else if(t->phase==T3_ALIGN || t->phase==T3_START_ALIGN)
   app()->heading.angle=CarMath_WrapDegrees(app()->heading.angle+(app()->route.targets[1]-app()->route.targets[0])*.01f/140*57.2957795131f);
  int counts[2];
  for(unsigned j=0;j<2;j++) {
   float cpr=j?DPT_RIGHT_CPR:DPT_LEFT_CPR;
   assert(app()->route.targets[j]>=0 && app()->route.targets[j]<=280.01f);
   residue[j]+=app()->route.targets[j]*.01f*cpr/(DPT_CIRC_M*1000);
   counts[j]=(int)residue[j];residue[j]-=counts[j];
  }
  step(10,counts[0],counts[1]);assert(a>=0&&b<=0);
 }
 assert(app()->runtime.state==DONE || app()->runtime.state==TIMEOUT);
 return app()->route.elapsed_ms;
}
static void until_lap_phase(unsigned lap,CarTask3Phase phase)
{
 for(unsigned k=0;k<12000 && app()->runtime.state==RUNNING &&
     !(app()->route.task4.completed_laps==lap && app()->route.task3.phase==phase);k++)step_route();
 assert(app()->runtime.state==RUNNING && app()->route.task4.completed_laps==lap && app()->route.task3.phase==phase);
}
int main(void)
{
 CarTask3Config c=CarTask4_Config();assert(CarTask3_ConfigValid(&c) && c.deadline_ms==80000U);
 assert(CarRoute_Task3Config().deadline_ms==40000U);
 task4_select();hold();assert(app()->button.selected_motion==MOVE_ROUTE);
 setup();queue(0,"TASK4!");step(10,0,0);assert(app()->button.selected_motion==MOVE_TASK4 && app()->runtime.state!=RUNNING);
 enable();press_key();assert(app()->runtime.state!=RUNNING); /* Cannot start without CAL. */
 task4_start();float reference=app()->route.task3.start_heading_deg;
 uint32_t started=app()->route.start_ms,samples=app()->heading.samples;
 unsigned initial_turn_entries=0,a_joins=0;CarTask3Phase last=T3_IDLE;
 for(unsigned k=0;k<12000 && app()->runtime.state==RUNNING;k++) {
  CarTask3 *t=&app()->route.task3;
  if(t->phase==T3_START_ALIGN && last!=T3_START_ALIGN) {
   ++initial_turn_entries;
   if(app()->route.task4.completed_laps>0) { ++a_joins;assert(a_joins==app()->route.task4.completed_laps && a==0 && b==0); }
  }
  assert(t->start_heading_deg==reference && app()->route.start_ms==started && app()->heading.samples==samples);
  assert(app()->route.task4.completed_laps<4);
  for(unsigned j=0;j<4;j++)assert(strlen(oled_lines[j])<=21);
  last=t->phase;step_route();
 }
 assert(app()->runtime.state==DONE && app()->route.task3.phase==T3_DONE && app()->route.task4.completed_laps==4);
 assert(initial_turn_entries==4 && a_joins==3 && signals==17 && a==0 && b==0);
 assert(app()->route.elapsed_ms<160000U && app()->route.distance_mm[0]>20000 && app()->route.distance_mm[1]>20000);
 app()->ui.render_ms=ms-200;step(10,0,0);assert(strstr(oled_lines[2],"DONE 4/4 DA"));
 for(unsigned k=0;k<50;k++)step(10,0,0);assert(signals==17 && a==0 && b==0);
 uint32_t model_ms=target_follow_four(0);
 assert(app()->runtime.state==DONE && app()->route.task4.completed_laps==4 && signals==17 && model_ms<160000U);
 printf("IDEAL four-lap target-following schedule: %lu ms; hypothetical track=140mm, not field acceptance.\n",(unsigned long)model_ms);
 /* Three A joins must settle; moving wheels cannot advance the lap. */
 task4_start();until_lap_phase(0,T3_FINAL_BRAKE);
 for(unsigned k=0;k<10;k++)step(10,177,176);
 assert(app()->route.task4.completed_laps==0 && app()->route.task3.phase==T3_FINAL_BRAKE);
 for(unsigned k=0;k<30;k++)step(10,0,0);
 assert(app()->runtime.state==RUNNING && app()->route.task4.completed_laps==1 && app()->route.task3.leg==0);
 /* Timeout is total-run based, including later laps; never report DONE. */
 task4_start();until_lap_phase(1,T3_START_ALIGN);app()->route.start_ms=ms-160000U;
 step(10,0,0);assert(app()->runtime.state==TIMEOUT && app()->route.task3.fault==T3_TIMEOUT && app()->route.task4.completed_laps==1 && a==0 && b==0);
 /* Faults in later laps stop without consuming laps or restarting. */
 for(unsigned k=0;k<8;k++) {
  task4_start();until_lap_phase(1,T3_ARC);
  switch(k) {
   case 0:off=1;step(10,0,0);assert(app()->runtime.state==OFF);break;
   case 1:queue(0,"STOP!TASK4!");step(10,0,0);assert(app()->runtime.state==STOPPED);break;
   case 2:imu_fail=1;step(10,0,0);assert(app()->runtime.state==YAW_BAD);break;
   case 3:adc_fail=1;app()->power.sample_ms=ms-100;step(10,0,0);assert(app()->runtime.state==ADC_BAD);break;
   case 4:faults[0]=1;step(10,0,0);assert(app()->runtime.state==UART_BAD);break;
   case 5:step(30,0,0);assert(app()->runtime.state==GAP);break;
   case 6:step(10,2000,0);assert(app()->runtime.state==FAST);break;
   case 7:line_inputs=15;for(unsigned n=0;n<11;n++)step(10,0,0);assert(app()->runtime.state==LINE_LOST && app()->route.task3.fault==T3_LOST);break;
  }
  assert(app()->route.task4.completed_laps==1 && app()->route.task3.phase==T3_FAULT && a==0 && b==0);
  TestState stopped=app()->runtime.state;for(unsigned j=0;j<30;j++)step(10,0,0);
  assert(app()->runtime.state==stopped && app()->route.task4.completed_laps==1 && a==0 && b==0);
 }
 /* Fresh restart clears count/snapshot and makes the initial AB->AC turn again. */
 task4_start();until_lap_phase(1,T3_ARC);line_inputs=15;for(unsigned k=0;k<11;k++)step(10,0,0);
 assert(app()->route.task3_trace.lost_seen);queue(0,"TASK4!");step(10,0,0);
 assert(app()->runtime.state==RUNNING && app()->route.task4.completed_laps==0 && app()->route.task3.phase==T3_START_ALIGN && !app()->route.task3_trace.lost_seen);
 /* Busy press cannot change task/queue a next lap. */
 task4_start();release_key();press_key();assert(app()->runtime.state==STOPPED && app()->button.selected_motion==MOVE_TASK4);
 /* Both-port STOP priority and TX drops remain safe in Task4. */
 task4_select();calibrate();assert(DualPITest_EnableBluetooth(&uart_mock)==HAL_OK);enable();press_key();
 queue(1,"STOP!");queue(0,"TASK4!");step(10,0,0);assert(app()->runtime.state==STOPPED && a==0 && b==0);
 task4_start();full_tx=1;for(unsigned k=0;k<100;k++)step_route();assert(app()->runtime.state==RUNNING);
 /* Millis and encoders wrap during four laps without resetting elapsed time. */
 task4_select();calibrate();ms=UINT32_MAX-15000U;app()->runtime.old_ms=ms;app()->line.sample_ms=ms;app()->power.sample_ms=ms;
 enable();press_key();for(unsigned k=0;k<12000 && app()->runtime.state==RUNNING;k++)step_route();
 assert(app()->runtime.state==DONE && signals==17 && app()->route.task4.completed_laps==4 && app()->route.elapsed_ms<160000U);
 /* The wrapper owns total elapsed time and rejects a second Start while active. */
 task4_start();CarTask3Input input=CarTask3Port_Input(app(),ms,1,0,0);CarTask3Output output;
 c=CarTask4_Config();assert(!CarTask4_Start(&app()->route.task4,&app()->route.task3,&c,&input));
 app()->route.task4.start_ms=ms-160000U;CarTask4_Update(&app()->route.task4,&app()->route.task3,&input,&output);
 assert(app()->route.task3.phase==T3_FAULT && app()->route.task3.fault==T3_TIMEOUT && !app()->route.task4.active && output.control==T3_HALT);
 /* Endpoint yaw must not rebase the fixed AC reference at the next A turn. */
 task4_start();until_lap_phase(0,T3_FINAL_BRAKE);float ac=app()->route.task4.ac_reference_deg;app()->heading.angle=8;
 for(unsigned k=0;k<30 && app()->route.task4.completed_laps==0;k++)step(10,0,0);
 assert(app()->route.task4.completed_laps==1 && app()->route.task3.phase==T3_START_ALIGN && app()->route.task3.start_heading_deg==ac);
 assert(!app()->route.task3.pending_start_signal && signals==5 && app()->route.task4.active);
 puts("PASS: Task4 four laps, three settled automatic A joins, fixed gyro reference/total deadline, 17 signals, mode/commands/UI, later-lap protections/STOP priority, restart and wrap. Mock software only.");
 return 0;
}
