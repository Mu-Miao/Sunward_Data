#include "car_task3_port.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static int a, b, clear_count, signal_count;
static void drive(int aa, int bb) { a=aa; b=bb; }
void CarWheel_Clear(CarAppContext *app)
{
    ++clear_count;
    for(unsigned i=0;i<2;i++) { app->wheel.items[i].command=0; app->wheel.items[i].integral=0; }
}
void CarSignal_Set(CarAppContext *app, int on) { app->signal.active=on; ++signal_count; }
void CarLine_SetTargets(CarAppContext *app, uint32_t now, float base)
{
    (void)now;
    app->route.targets[LEFT]=base+20.0f;
    app->route.targets[RIGHT]=base-20.0f;
}
int main(void)
{
    CarAppContext app={0};
    app.runtime.drive=drive; app.runtime.state=RUNNING;
    app.heading.phase=Y_READY; app.heading.angle=170.0f;
    app.line.sample_ms=UINT32_MAX-2U;
    app.line.valid=1; app.line.count=2; app.line.black[1]=app.line.black[2]=1;
    app.route.distance_mm[LEFT]=100; app.route.distance_mm[RIGHT]=102;
    CarTask3Input i=CarTask3Port_Input(&app,7U,1,0,0);
    assert(i.sample_age_ms==10 && i.centered && i.distance_mm==101 && i.heading_ready);
    CarTask3Output o={.control=T3_HOLD_HEADING,.heading_deg=-175,.base_mmps=50};
    CarTask3Port_Apply(&app,&o,7U);
    assert(app.heading.error==15 && app.route.targets[LEFT]==35 && app.route.targets[RIGHT]==65);
    o.control=T3_TURN_FORWARD; o.left_mmps=0; o.right_mmps=60;
    CarTask3Port_Apply(&app,&o,8U);
    assert(app.route.targets[LEFT]==0 && app.route.targets[RIGHT]==60);
    o.control=T3_FOLLOW_LINE; o.base_mmps=200;
    CarTask3Port_Apply(&app,&o,9U);
    assert(app.route.targets[LEFT]==220 && app.route.targets[RIGHT]==180);
    o.control=T3_HALT; o.signal_point=T3_POINT_C; o.reset_controllers=1;
    CarTask3Port_Apply(&app,&o,10U);
    assert(a==0 && b==0 && clear_count==1 && signal_count==1 && app.signal.ms==10U);
    app.runtime.state=LOW; o.control=T3_TURN_FORWARD;
    CarTask3Port_Apply(&app,&o,11U);
    assert(a==0 && b==0 && app.route.targets[LEFT]==0 && app.route.targets[RIGHT]==0 && signal_count==1);
    assert(CarTask3Port_StopState(T3_TIMEOUT)==TIMEOUT && CarTask3Port_StopState(T3_SETTLE)==SETTLE_BAD);
    puts("PASS: T13 adapter snapshot, heading sign/wrap, zero-wheel turn targets, line reuse, brake/signal and fault dominance. No PWM commit.");
    return 0;
}
