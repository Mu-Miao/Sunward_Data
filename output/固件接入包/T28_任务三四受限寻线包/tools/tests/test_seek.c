/* Default production search behaviour and actual scheduler integration. No hardware access. */
#define CAR_TASK3_TEST_MAIN legacy_task3_main
#include "test_task3.c"
#undef CAR_TASK3_TEST_MAIN

static void seek_start(unsigned task)
{
 if(task==3) {task3_start();app()->route.task3.config=CarRoute_Task3Config();}
 else {
  setup();line_inputs=15;hold();hold();hold();hold();calibrate();enable();press_key();
  assert(app()->route.motion==MOVE_TASK4 && app()->runtime.state==RUNNING);
  assert(app()->route.task3.config.white_seek_ms==300 && app()->route.task3.config.vertex_seek_ms==3000);
 }
}
static void seek_arc(unsigned task)
{
 seek_start(task);reach_phase(T3_ARC);line_inputs=9;step(10,177,176);
 assert(app()->route.task3.arc_valid_seen);
}
static void seek_d(void)
{
 seek_start(3);
 for(unsigned n=0;n<3500 && !(app()->route.task3.leg==2 && app()->route.task3.phase==T3_DIAGONAL);n++) {
  assert(app()->runtime.state==RUNNING);step_route();
 }
 assert(app()->route.task3.phase==T3_DIAGONAL && app()->route.task3.leg==2);
 app()->route.distance_mm[0]=app()->route.distance_mm[1]=app()->route.task3.leg_start_mm+1281;
 line_inputs=15;step(10,0,0);
 assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_VERTEX_SEEK && a==0 && b==0);
}
int main(void)
{
 CarTask3Config c=CarRoute_Task3Config();assert(CarTask3_ConfigValid(&c));
 c.white_seek_ms=301;assert(!CarTask3_ConfigValid(&c));
 c=CarRoute_Task3Config();c.white_seek_mm=21;assert(!CarTask3_ConfigValid(&c));
 c=CarRoute_Task3Config();c.white_seek_deg=13;assert(!CarTask3_ConfigValid(&c));
 c=CarRoute_Task3Config();c.vertex_seek_ms=3001;assert(!CarTask3_ConfigValid(&c));
 c=CarRoute_Task3Config();c.vertex_seek_mm=NAN;assert(!CarTask3_ConfigValid(&c));
 /* Full single/four-lap route under actual default config, without field dynamics claims. */
 for(unsigned task=3;task<=4;task++) {
  seek_start(task);
  for(unsigned n=0;n<12500 && app()->runtime.state==RUNNING;n++)step_route();
  assert(app()->runtime.state==DONE && a==0 && b==0);
  assert(signals==(task==3?5:17));
  if(task==4)assert(app()->route.task4.completed_laps==4);
 }
 /* The reported D-side miss now enters bounded search, never fabricates a D signal. */
 seek_d();int before=signals;
 step(10,0,0);assert(app()->route.targets[0]>0 && app()->route.targets[1]==0 && signals==before);
 line_inputs=11;
 for(unsigned n=0;n<3;n++){step(10,0,0);assert(app()->route.task3.phase==T3_VERTEX_SEEK && a==0 && b==0 && signals==before);}
 step(10,0,0);assert(app()->route.task3.phase==T3_VERTEX_BRAKE && signals==before+1);
 for(unsigned n=0;n<500 && app()->route.task3.phase!=T3_ARC;n++)step_route();
 assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_ARC && app()->route.task3.leg==3);
 /* Search is terminated by original global vertex window, local time/distance and yaw. */
 for(unsigned limit=0;limit<4;limit++) {
  seek_d();before=signals;
  switch(limit) {
   case 0:app()->route.distance_mm[0]=app()->route.distance_mm[1]=app()->route.task3.leg_start_mm+1401;break;
   case 1:app()->route.task3.phase_ms=ms-3000;break;
   case 2:app()->route.task3.phase_start_mm-=80;break;
   case 3:app()->heading.angle+=60;break;
  }
  step(10,0,0);assert(app()->runtime.state!=RUNNING && a==0 && b==0 && signals==before);
 }
 /* White vertex search aligns and settles before advancing, never reverses. */
 seek_d();
 /* Use production tangent formula directly for D (initial AB reference +180 degrees). */
 app()->heading.angle=CarMath_WrapDegrees(app()->route.task3.start_heading_deg+app()->route.task3.diagonal_angle_deg+180);
 for(unsigned n=0;n<21;n++){step(10,0,0);assert(a==0 && b==0);}
 assert(app()->route.task3.seek_aligned);step(10,0,0);
 assert(a>0 && b<0 && app()->route.targets[0]>0 && app()->route.targets[1]>0);
 /* Both turns must remain halted while wheels still move or yaw leaves tolerance. */
 seek_start(3);reach_phase(T3_ALIGN_SETTLE);uint32_t turn_start=app()->route.task3.phase_ms;
 for(unsigned n=0;n<10;n++){step(10,177,176);assert(app()->route.task3.phase==T3_ALIGN_SETTLE && a==0 && b==0);}
 app()->heading.angle+=5;step(10,0,0);
 assert(app()->route.task3.phase==T3_ALIGN && app()->route.task3.phase_ms==turn_start && a==0 && b==0);
 /* Transient all-white arc interval bridges slowly while preserving valid=false and fixed budgets. */
 for(unsigned task=3;task<=4;task++) {
  seek_arc(task);uint32_t start=app()->route.start_ms;float origin=app()->route.task3.leg_start_mm;
  line_inputs=15;step(10,0,0);
  assert(!app()->line.valid && app()->route.task3.loss_seek && a>0 && b<0);
  assert(app()->route.base==40 && app()->route.targets[0]<=56 && app()->route.targets[1]<=56);
  uint32_t loss=app()->route.task3_trace.lost_ms;
  for(unsigned n=0;n<12;n++)step(10,100,100);
  assert(app()->runtime.state==RUNNING && app()->route.task3_trace.lost_ms==loss);
  line_inputs=9;
  for(unsigned n=0;n<3;n++){step(10,0,0);assert(a==0 && b==0 && app()->route.task3_trace.lost_seen);}
  step(10,0,0);assert(app()->runtime.state==RUNNING && !app()->route.task3.lost_seen && !app()->route.task3_trace.lost_seen);
  assert(app()->route.base<=50.01f && app()->route.start_ms==start && app()->route.task3.leg_start_mm==origin);
 }
 /* An unconfirmed opposite-side glitch cannot replace the last trusted steering direction. */
 seek_arc(3);line_inputs=14;step(10,177,176);
 assert(app()->route.task3_trace.valid_error>0);
 line_inputs=15;step(10,0,0);assert(app()->route.targets[0]>app()->route.targets[1]);
 line_inputs=7;step(10,0,0);assert(app()->line.error<0 && a==0 && b==0);
 line_inputs=15;step(10,0,0);
 assert(app()->route.task3_trace.valid_error>0 && app()->route.targets[0]>app()->route.targets[1]);
 /* Every bound is enforced, including 300ms even when the late sample is valid. */
 for(unsigned limit=0;limit<4;limit++) {
  seek_arc(3);line_inputs=15;step(10,0,0);
  if(limit==0 || limit==3) {
   for(unsigned n=0;n<29;n++)step(10,0,0);
   if(limit==3)line_inputs=9;
  } else if(limit==1)app()->route.task3.lost_start_mm-=20;
  else app()->heading.angle+=13;
  step(10,0,0);assert(app()->runtime.state==LINE_LOST && a==0 && b==0 && app()->route.task3.fault==T3_LOST);
  line_inputs=9;for(unsigned n=0;n<20;n++)step(10,0,0);
  assert(app()->runtime.state==LINE_LOST && a==0 && b==0);
 }
 /* No valid history, all-black and separated-black patterns never authorize positive search. */
 for(unsigned pattern=0;pattern<3;pattern++) {
  seek_start(3);reach_phase(T3_ARC);
  if(pattern) {line_inputs=9;step(10,0,0);}
  line_inputs=pattern==0?15:(pattern==1?0:5);step(10,0,0);
  assert(a==0 && b==0 && !app()->route.task3.loss_seek);
  for(unsigned n=0;n<10;n++)step(10,0,0);
  assert(app()->runtime.state==LINE_LOST);
 }
 /* Alternating white/valid samples do not renew the search episode. */
 seek_arc(3);line_inputs=15;step(10,0,0);uint32_t loss=app()->route.task3.lost_start_ms;
 for(unsigned n=0;n<29;n++) {line_inputs=n%2?15:9;step(10,0,0);assert(app()->route.task3.lost_start_ms==loss);}
 step(10,0,0);assert(app()->runtime.state==LINE_LOST && a==0 && b==0);
 /* STOP, enable, sensor, scheduler and total watchdog override either search phase. */
 for(unsigned phase=0;phase<2;phase++)for(unsigned f=0;f<7;f++) {
  if(phase)seek_d();else {seek_arc(3);line_inputs=15;step(10,0,0);}
  switch(f) {
   case 0:off=1;break;
   case 1:queue(0,"STOP!");break;
   case 2:imu_fail=1;break;
   case 3:adc_fail=1;app()->power.sample_ms=ms-100;break;
   case 4:faults[0]=1;break;
   case 5:app()->route.start_ms=ms-40000;break;
   case 6:step(30,0,0);break;
  }
  step(10,0,0);assert(app()->runtime.state!=RUNNING && a==0 && b==0);
 }
 puts("PASS: default Task3/4 completion, D miss bounded acquisition without false point signals, settled turn/yaw recheck, all-white bridge/reacquisition, all bounds, invalid/no-history rejection, chatter deadline and fault priority. Software only.");
 return 0;
}
