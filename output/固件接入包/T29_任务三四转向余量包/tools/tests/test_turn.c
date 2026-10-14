/* Reduced stationary turning: keep geometric references, settle gates, and directional capture. */
#define CAR_TASK3_TEST_MAIN legacy_task3_main
#include "test_task3.c"
#undef CAR_TASK3_TEST_MAIN

static float nominal_heading(unsigned leg)
{
 CarTask3 *t=&app()->route.task3;
 return CarMath_WrapDegrees(t->start_heading_deg+
  (leg==2?180+2*t->diagonal_angle_deg:t->diagonal_angle_deg+(leg==3?180:0)));
}
static void turn_start(unsigned mode)
{
 if(mode==3)task3_start();
 else {setup();line_inputs=15;hold();hold();hold();hold();calibrate();enable();press_key();}
 assert(app()->runtime.state==RUNNING && app()->route.task3.config.turn_relief_deg==5);
}
static void settle_at(float angle,CarTask3Phase aligning,CarTask3Phase settled)
{
 app()->heading.angle=angle;
 for(unsigned n=0;n<10 && app()->route.task3.phase==aligning;n++) {
  step(10,0,0);assert(a==0 && b==0);
 }
 assert(app()->route.task3.phase==settled);
 for(unsigned n=0;n<30 && app()->route.task3.phase==settled;n++) {
  step(10,0,0);assert(a==0 && b==0);
 }
 assert(app()->runtime.state==RUNNING && app()->route.task3.phase!=settled);
}
static void gentle_diagonal(float reference)
{
 line_inputs=15;step(10,0,0);
 float first=fabsf(CarMath_WrapDegrees(reference-app()->heading.angle));
 assert(first>4.9f && first<5.1f);
 assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-reference))<.001f);
 for(unsigned n=0;n<100;n++) {
  float left=app()->route.targets[0],right=app()->route.targets[1];
  assert(left>0 && right>0);
  /* Hypothetical 140mm ideal wheel plant, not a vehicle acceptance model. */
  app()->heading.angle=CarMath_WrapDegrees(app()->heading.angle+(right-left)*.01f/140*57.2957795131f);
  int dl=CarMath_Round(left*.01f*DPT_LEFT_CPR/(DPT_CIRC_M*1000));
  int dr=CarMath_Round(right*.01f*DPT_RIGHT_CPR/(DPT_CIRC_M*1000));
  step(10,dl,dr);
  assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_DIAGONAL && a>=0 && b<=0);
 }
 assert(fabsf(CarMath_WrapDegrees(reference-app()->heading.angle))<first*.5f);
}
int main(void)
{
 CarTask3Config c=CarRoute_Task3Config();assert(c.turn_relief_deg==5 && CarTask3_ConfigValid(&c));
 c.turn_relief_deg=NAN;assert(!CarTask3_ConfigValid(&c));
 c.turn_relief_deg=-1;assert(!CarTask3_ConfigValid(&c));
 c.turn_relief_deg=11;assert(!CarTask3_ConfigValid(&c));
 c=CarRoute_Task3Config();c.radius_mm=1;assert(!CarTask3_ConfigValid(&c));
 c=CarRoute_Task3Config();c.turn_relief_deg=0;assert(CarTask3_ConfigValid(&c));
 assert(CarTask3_DefaultConfig().turn_relief_deg==0);
 /* Both modes retain the exact AC reference while stationary A turning ends five degrees early. */
 for(unsigned mode=3;mode<=4;mode++) {
  turn_start(mode);float ac=app()->route.task3.start_heading_deg;
  assert(fabsf(ac+38.65981f)<.001f);
  float partial=CarMath_WrapDegrees(ac+5);
  assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-partial))<.001f);
  settle_at(partial,T3_START_ALIGN,T3_START_SETTLE);
  assert(app()->route.task3.phase==T3_DIAGONAL && app()->route.task3.start_heading_deg==ac);
  if(mode==4)assert(app()->route.task4.ac_reference_deg==ac);
  gentle_diagonal(ac);
 }
 /* C left, B left, D right: verify reduced target, halt/debounce/settle and correct next control. */
 for(unsigned mode=3;mode<=4;mode++)for(unsigned leg=1;leg<=3;leg++) {
  turn_start(mode);float ac=app()->route.task3.start_heading_deg;
  for(unsigned n=0;n<3500 && !(app()->route.task3.leg==leg && app()->route.task3.phase==T3_ALIGN);n++) {
   assert(app()->runtime.state==RUNNING);step_route();
  }
  assert(app()->route.task3.leg==leg && app()->route.task3.phase==T3_ALIGN);
  float nominal=nominal_heading(leg),partial=CarMath_WrapDegrees(nominal+(leg==3?5:-5));
  step(10,0,0);
  assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-partial))<.001f);
  settle_at(partial,T3_ALIGN,T3_ALIGN_SETTLE);
  assert(app()->route.task3.start_heading_deg==ac);
  if(mode==4)assert(app()->route.task4.ac_reference_deg==ac);
  if(leg==2) {
   assert(app()->route.task3.phase==T3_DIAGONAL);
   line_inputs=15;step(10,0,0);
   assert(app()->route.targets[1]>app()->route.targets[0]);
   gentle_diagonal(nominal);
  } else {
   assert(app()->route.task3.phase==T3_CAPTURE);
   line_inputs=15;step(10,0,0);
   assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-partial))<.001f);
   assert(app()->route.targets[0]>0 && app()->route.targets[1]>0);
   line_inputs=leg==1?13:11;
   for(unsigned n=0;n<4;n++)step(10,100,100);
   assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_ARC);
  }
 }
 /* Zero relief reproduces full geometric alignment, permitting a simple parameter rollback. */
 turn_start(3);app()->route.task3.config.turn_relief_deg=0;step(10,0,0);
 assert(app()->heading.reference==app()->route.task3.start_heading_deg);
 puts("PASS: Task3/4 A/B/C/D 5-degree reduced stationary targets, fixed AC reference, nominal diagonal feedback convergence, low-speed off-tangent line capture, settle/heading gates, parameter validation and zero-relief fallback. Software only.");
 return 0;
}
