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
 case T3_ALIGN: case T3_START_ALIGN: {
  float target=t->phase==T3_START_ALIGN?t->start_heading_deg:join_target();
  float e=CarMath_WrapDegrees(target-app()->heading.angle);
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
 if(app()->runtime.state==RUNNING && (app()->route.task3.phase==T3_ALIGN||app()->route.task3.phase==T3_START_ALIGN)) {
  if(app()->route.targets[0]==0)assert(app()->wheel.items[0].command==0);
  if(app()->route.targets[1]==0)assert(app()->wheel.items[1].command==0);
 }
}
/* Ideal target-following encoder/yaw inputs, with a hypothetical 140mm track.
   This evaluates the schedule; it is not a motor/tyre/track dynamics model. */
static uint32_t target_follow_lap(int prior_profile)
{
 task3_start();
 if(prior_profile) {
  app()->route.task3.config.diagonal_mmps=200;
  app()->route.task3.config.turn_mmps=60;
  app()->route.task3.config.acceleration_mmps2=100;
  app()->route.task3.config.arc_acceleration_mmps2=100;
  app()->route.task3.config.approach_mm=150;
  app()->route.task3.config.minimum_mmps=40;
 }
 float residue[2]={0,0};
 for(unsigned k=0;k<4500 && app()->runtime.state==RUNNING;k++) {
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
static void reach_phase(CarTask3Phase phase)
{
 for(unsigned i=0;i<2500&&app()->runtime.state==RUNNING&&app()->route.task3.phase!=phase;i++)step_route();
 assert(app()->runtime.state==RUNNING && app()->route.task3.phase==phase);
}
#ifndef CAR_TASK3_TEST_MAIN
#define CAR_TASK3_TEST_MAIN main
#endif
int CAR_TASK3_TEST_MAIN(void)
{
 task3_start();
 assert(app()->route.task3.phase==T3_START_ALIGN && app()->route.task3.leg==0);
 assert(fabsf(app()->route.task3.start_heading_deg + 38.65981f)<0.01f);
 assert(app()->route.targets[0]>0 && app()->route.targets[1]==0 && a==0 && b<0);
 assert(app()->route.goal>5070 && app()->route.goal<5080);
 uint32_t start=app()->route.start_ms;
 cmd("TASK3!");assert(app()->route.start_ms==start);
 for(unsigned i=0;i<3500&&app()->runtime.state==RUNNING;i++)step_route();
 assert(app()->runtime.state==DONE && app()->route.task3.phase==T3_DONE && signals==5 && a==0 && b==0);
 assert(app()->route.elapsed_ms<40000 && app()->route.leg==3U && rp[0]==wp[0] && wp[1]==0);
 for(unsigned i=0;i<50;i++)step(10,0,0);
 assert(signals==5 && !app()->signal.active && strcmp(oled_lines[0],"TASK 3 CAP V2")==0);
 assert(strstr(oled_lines[2],"DONE DA T"));for(unsigned i=0;i<4;i++)assert(strlen(oled_lines[i])<=21);
 /* Task-specific faster schedule, with unchanged half-circle speed/deadline. */
 assert(CarRoute_Task3Config().diagonal_mmps==240 && CarRoute_Task3Config().arc_mmps==200);
 assert(CarRoute_Task3Config().turn_mmps==80 && CarRoute_Task3Config().capture_mmps==50);
 assert(CarRoute_Task3Config().deadline_ms==40000U);
 uint32_t old_schedule=target_follow_lap(1),new_schedule=target_follow_lap(0);
 assert(app()->runtime.state==DONE && signals==5 && new_schedule<36000U);
 assert(new_schedule+4000U<old_schedule);
 printf("IDEAL target-following schedule: prior=%lu ms revised=%lu ms; hypothetical track=140mm, not field acceptance.\n",(unsigned long)old_schedule,(unsigned long)new_schedule);
 /* Exact observed DA 993mm at 40 seconds remains a timeout, never false DONE. */
 task3_start();for(unsigned k=0;k<3500&&app()->runtime.state==RUNNING && !(app()->route.task3.phase==T3_ARC && app()->route.task3.leg==3);k++)step_route();
 assert(app()->route.task3.phase==T3_ARC && app()->route.task3.leg==3);
 app()->route.distance_mm[0]=app()->route.distance_mm[1]=app()->route.task3.leg_start_mm+993;
 app()->route.start_ms=ms-40000U;step(10,0,0);
 assert(app()->runtime.state==TIMEOUT && app()->route.task3.fault==T3_TIMEOUT && fabsf(progress()-993)<.01f && a==0 && b==0);
 /* Arc ramp remains independent from faster diagonals, both CB and DA. */
 for(unsigned leg=1;leg<=3;leg+=2) {
  task3_start();for(unsigned k=0;k<3500&&app()->runtime.state==RUNNING && !(app()->route.task3.phase==T3_ARC && app()->route.task3.leg==leg);k++)step_route();
  assert(app()->route.task3.phase==T3_ARC && app()->route.task3.leg==leg);
  assert(app()->route.task3.config.acceleration_mmps2==150 && app()->route.task3.config.arc_acceleration_mmps2==100);
  /* At 182mm and 1s after entering the arc: 50+100*1.01=151, not 200. */
  line_inputs=9;app()->route.task3.phase_ms=ms-1000;
  app()->route.distance_mm[0]=app()->route.distance_mm[1]=app()->route.task3.leg_start_mm+182;
  step(10,0,0);assert(app()->runtime.state==RUNNING && fabsf(app()->route.base-151)<.05f);
  assert(app()->route.targets[0]<152 && app()->route.targets[1]<152);
  /* Invalid sensors still stop on the first sample and latch F7 at 100ms. */
  line_inputs=15;step(10,0,0);assert(a==0 && b==0 && app()->runtime.state==RUNNING);
  for(unsigned k=0;k<10;k++)step(10,0,0);
  assert(app()->runtime.state==LINE_LOST && app()->route.task3.fault==T3_LOST && fabsf(progress()-182)<.01f && a==0 && b==0);
 }
 /* A faster diagonal ramp does not bleed into an arc, and invalid new config fails closed. */
 CarTask3Config c=CarRoute_Task3Config();c.arc_acceleration_mmps2=0;assert(!CarTask3_ConfigValid(&c));
 c=CarRoute_Task3Config();c.arc_acceleration_mmps2=NAN;assert(!CarTask3_ConfigValid(&c));
 task3_start();reach_phase(T3_DIAGONAL);line_inputs=15;app()->route.task3.phase_ms=ms-1000;
 step(10,0,0);assert(fabsf(app()->route.base-201.5f)<.05f);
 /* Busy new button press cancels; held press cannot restart. */
 task3_start();release_key();press_key();assert(app()->runtime.state==STOPPED && a==0 && b==0 && app()->route.task3.phase==T3_FAULT);
 for(unsigned i=0;i<200;i++)step(10,0,0);assert(app()->runtime.state==STOPPED);
 release_key();press_key();assert(app()->runtime.state==RUNNING && app()->route.task3.leg==0);
 /* Startup waits for both gyro convergence and settled wheels, excluding pivot travel. */
 task3_start();reach_phase(T3_START_SETTLE);
 assert(app()->route.leg==0U && app()->route.task3.leg_start_mm==0 && app()->route.distance_mm[0]>0);
 uint32_t before_ac=app()->route.start_ms;
 for(unsigned i=0;i<25&&app()->route.task3.phase==T3_START_SETTLE;i++)step_route();
 assert(app()->route.task3.phase==T3_DIAGONAL && fabsf(progress())<0.01f && app()->route.start_ms==before_ac);
 /* Initial wrong heading/speed or yaw drift cannot bypass the settle gate. */
 task3_start();reach_phase(T3_START_SETTLE);app()->heading.angle+=5;step(10,0,0);
 assert(app()->route.task3.phase==T3_START_ALIGN && app()->runtime.state==RUNNING && a==0 && b==0);
 task3_start();app()->route.task3.phase_start_mm=-100;step(10,0,0);
 assert(app()->runtime.state==YAW_BAD && app()->route.task3.fault==T3_TURN);
 task3_start();for(unsigned i=0;i<410&&app()->runtime.state==RUNNING;i++)step(10,177,0);
 assert(app()->runtime.state==YAW_BAD && app()->route.task3.fault==T3_TURN && a==0 && b==0);
 /* Stop dominates the startup gyro turn. */
 task3_start();queue(0,"STOP!");step(10,0,0);assert(app()->runtime.state==STOPPED && a==0 && b==0);
 /* C capture: off-centre black must steer before centred entry, not hold yaw. */
 task3_start();reach_phase(T3_CAPTURE);line_inputs=11;step(10,0,0);
 assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_CAPTURE);
 assert(app()->route.targets[0]<app()->route.targets[1] && app()->route.targets[0]>0);
 line_inputs=13;step(10,0,0);
 assert(app()->route.targets[0]>app()->route.targets[1] && app()->route.targets[1]>0);
 line_inputs=9;for(unsigned i=0;i<4;i++)step(10,0,0);
 assert(app()->route.task3.phase==T3_ARC && fabsf(progress())<0.01f);
 /* White approach remains heading-controlled and strictly bounded. */
 task3_start();reach_phase(T3_CAPTURE);line_inputs=15;step(10,0,0);
 assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_CAPTURE && app()->route.targets[0]>0 && app()->route.targets[1]>0);
 app()->route.task3.phase_ms=ms-app()->route.task3.config.capture_timeout_ms;
 step(10,0,0);assert(app()->runtime.state==LINE_BAD && app()->route.task3.fault==T3_EDGE && a==0 && b==0);
 /* User display LINE CB F6 S1319 can be capture overtravel, not arc overtravel. */
 task3_start();reach_phase(T3_CAPTURE);line_inputs=11;
 app()->route.task3.leg_start_mm=0;app()->route.task3.phase_start_mm=1258;
 app()->route.distance_mm[0]=app()->route.distance_mm[1]=1319;
 step(10,0,0);assert(app()->runtime.state==LINE_BAD && app()->route.task3.fault==T3_EDGE && fabsf(progress()-1319)<0.01f);
 float stopped_progress=progress();unsigned stopped_fault=app()->route.task3.fault;
 step(10,700,650);assert(progress()==stopped_progress && app()->route.task3.fault==stopped_fault && a==0 && b==0);
 /* Keep valid but uncentred capture inside the same 60mm limit. */
 task3_start();reach_phase(T3_CAPTURE);line_inputs=13;
 app()->route.distance_mm[0]=app()->route.distance_mm[1]=app()->route.task3.phase_start_mm+61;
 step(10,0,0);assert(app()->runtime.state==LINE_BAD && app()->route.task3.fault==T3_EDGE && a==0 && b==0);
 /* Turns bypass the legacy 20deg heading trip, zero wheel clears old integral. */
 task3_start();reach_phase(T3_ALIGN);app()->wheel.items[0].integral=1700;step(10,0,0);
 assert(app()->runtime.state==RUNNING && app()->route.targets[0]==0 && app()->wheel.items[0].integral==0 && b==0 && a>0);
 /* D capture uses the same line feedback without reversing either wheel. */
 task3_start();for(unsigned i=0;i<3500&&app()->runtime.state==RUNNING && !(app()->route.task3.phase==T3_CAPTURE && app()->route.task3.leg==3U);i++)step_route();
 assert(app()->route.task3.phase==T3_CAPTURE && app()->route.task3.leg==3U);
 line_inputs=11;step(10,0,0);assert(app()->route.targets[0]<app()->route.targets[1] && app()->route.targets[0]>0);
 line_inputs=9;for(unsigned i=0;i<4;i++)step(10,0,0);assert(app()->route.task3.phase==T3_ARC);
 /* D F6/S1308/Y173: valid single/three black channels need not become centred pairs. */
 for(unsigned leg=1;leg<=3;leg+=2) {
  for(unsigned pattern=0;pattern<2;pattern++) {
   task3_start();
   for(unsigned k=0;k<3500 && app()->runtime.state==RUNNING &&
       !(app()->route.task3.phase==T3_CAPTURE && app()->route.task3.leg==leg);k++)step_route();
   assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_CAPTURE);
   float origin=app()->route.task3.phase_start_mm;
   app()->heading.angle=join_target()+(leg==3?-7:7);
   line_inputs=pattern?8:13; /* Three contiguous black, or one black. Neither is centred. */
   for(unsigned k=0;k<3;k++) {
    step(10,150,145);
    assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_CAPTURE);
    assert(app()->route.targets[0]>0 && app()->route.targets[1]>0);
    assert(app()->route.targets[0]>app()->route.targets[1]);
   }
   step(10,150,145);
   assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_ARC);
   assert(fabsf(progress())<.01f && app()->route.task3.leg_start_mm>origin);
   step(10,150,145);assert(app()->runtime.state==RUNNING && app()->route.targets[0]>0 && app()->route.targets[1]>0);
  }
 }
 /* Loss of valid pattern resets acquisition stability; fresh 30ms is required. */
 task3_start();reach_phase(T3_CAPTURE);line_inputs=13;
 step(10,150,145);step(10,150,145);step(10,150,145);
 line_inputs=15;step(10,150,145);assert(app()->route.task3.phase==T3_CAPTURE && !app()->route.task3.edge_seen);
 line_inputs=13;for(unsigned k=0;k<3;k++) {step(10,150,145);assert(app()->route.task3.phase==T3_CAPTURE);}
 step(10,150,145);assert(app()->route.task3.phase==T3_ARC);
 /* All-black/non-contiguous patterns never acquire an arc. */
 for(unsigned k=0;k<2;k++) {
  task3_start();reach_phase(T3_CAPTURE);line_inputs=k?5:0;
  for(unsigned j=0;j<5;j++)step(10,0,0);
  assert(app()->route.task3.phase==T3_CAPTURE && !app()->route.task3.edge_seen);
 }
 /* A valid line still cannot override capture timeout or heading bounds. */
 task3_start();reach_phase(T3_CAPTURE);line_inputs=13;
 app()->route.task3.phase_ms=ms-app()->route.task3.config.capture_timeout_ms;
 step(10,0,0);assert(app()->runtime.state==LINE_BAD && app()->route.task3.fault==T3_EDGE && a==0 && b==0);
 task3_start();reach_phase(T3_CAPTURE);line_inputs=13;app()->heading.angle=join_target()+21;
 step(10,0,0);assert(app()->runtime.state==YAW_BAD && app()->route.task3.fault==T3_HEADING && a==0 && b==0);
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
 /* Frozen loss evidence survives OFF and physical movement, but recovers/restarts clear it. */
 task3_start();reach_phase(T3_ARC);line_inputs=11;step(10,177,176);
 assert(app()->route.task3_trace.valid_seen && app()->route.task3_trace.valid_raw[0]==1 && app()->route.task3_trace.valid_raw[2]==0);
 float old_targets[2]={app()->route.targets[0],app()->route.targets[1]};
 app()->heading.angle=-1;line_inputs=15;step(10,177,176);
 assert(a==0 && b==0 && app()->route.task3_trace.lost_seen && app()->runtime.state==RUNNING);
 assert(app()->route.task3_trace.lost_raw[0]==1 && app()->route.task3_trace.lost_raw[3]==1);
 assert(app()->route.task3_trace.targets[0]==old_targets[0] && app()->route.task3_trace.targets[1]==old_targets[1]);
 assert(app()->route.task3_trace.velocity[0]>55 && app()->route.task3_trace.velocity[1]>55);
 uint32_t loss_ms=app()->route.task3_trace.lost_ms;
 for(unsigned k=0;k<10;k++)step(10,0,0);
 assert(app()->runtime.state==LINE_LOST && app()->route.task3.fault==T3_LOST);
 off=1;line_inputs=0;app()->heading.angle=90;step(10,600,600);
 assert(app()->route.task3_trace.lost_ms==loss_ms && app()->route.task3_trace.lost_yaw==-1);
 assert(app()->route.task3_trace.lost_raw[0]==1 && app()->route.task3_trace.targets[0]==old_targets[0]);
 app()->ui.render_ms=ms-200;step(10,0,0);
 assert(strcmp(oled_lines[1],"LF1111 PRE1101")==0 && strstr(oled_lines[3],"Y-1"));
 for(unsigned k=0;k<205;k++)step(10,0,0);
 assert(strncmp(oled_lines[1],"E",1)==0 && strncmp(oled_lines[3],"T",1)==0 && strstr(oled_lines[3]," V"));
 for(unsigned j=0;j<4;j++)assert(strlen(oled_lines[j])<=21);
 /* A short invalid glitch recovers; final fault must freeze the later invalid pattern. */
 task3_start();reach_phase(T3_ARC);line_inputs=9;step(10,177,176);
 line_inputs=15;step(10,0,0);uint32_t first_loss=app()->route.task3_trace.lost_ms;
 line_inputs=13;step(10,0,0);assert(!app()->route.task3_trace.lost_seen);
 line_inputs=0;step(10,0,0);assert(app()->route.task3_trace.lost_ms!=first_loss && app()->route.task3_trace.lost_raw[0]==0);
 for(unsigned k=0;k<10;k++)step(10,0,0);assert(app()->runtime.state==LINE_LOST);
 queue(0,"TASK3!");step(10,0,0);assert(app()->runtime.state==RUNNING && !app()->route.task3_trace.lost_seen && !app()->route.task3_trace.valid_seen);
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
 puts("PASS: actual CarApp scheduler Task3 GPIO selection/start, initial AB->AC right gyro turn/settle, AC->CB->BD->DA settled completion, five signals, zero inner PI, no reverse commands, old heading gate isolation, STOP/OFF/RX/MPU/ADC/LOW/FAST/REV/STALL/GAP, line/turn/time guards, restart, telemetry congestion, encoder/millis wrap. Simulation only.");
 return 0;
}
