#include "car_task3.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static CarTask3 r;
static CarTask3Input in;
static CarTask3Output out;
static unsigned arrivals[5], starts;
static void tick(void)
{
    in.now_ms += 10U;
    CarTask3_Update(&r, &in, &out);
    assert(out.base_mmps >= 0.0f && out.left_mmps >= 0.0f && out.right_mmps >= 0.0f);
    if (out.signal_point) {
        if (out.start_signal) ++starts;
        else ++arrivals[out.signal_point];
    }
    if (r.phase == T3_FAULT || r.phase == T3_DONE) assert(out.control == T3_HALT);
}
static void ticks(unsigned n) { while (n--) tick(); }
static void line(unsigned count, unsigned valid, unsigned centered)
{
    in.black_count = count; in.line_valid = valid; in.centered = centered;
}
static void init(uint32_t now, float heading)
{
    memset(&r, 0, sizeof r); memset(&in, 0, sizeof in);
    memset(arrivals, 0, sizeof arrivals); starts = 0U;
    in.enabled = in.heading_ready = 1U;
    in.now_ms = now; in.yaw_deg = heading; in.distance_mm = 33.0f;
    CarTask3Config c = CarTask3_DefaultConfig();
    assert(CarTask3_Start(&r, &c, &in));
    assert(fabsf(r.diagonal_mm - 1280.6248f) < 0.02f);
    assert(fabsf(r.diagonal_angle_deg - 38.65981f) < 0.001f);
    tick(); assert(starts == 1U && out.control == T3_HOLD_HEADING);
}
static void arrive_diagonal(void)
{
    in.distance_mm = r.leg_start_mm + r.diagonal_mm;
    line(2, 1, 1);
    tick(); assert(out.control == T3_HALT && r.phase == T3_DIAGONAL);
    ticks(3); assert(r.phase == T3_VERTEX_BRAKE);
}
static void settle(void)
{
    in.left_mmps = in.right_mmps = 0.0f;
    ticks(21); assert(r.phase == T3_ALIGN);
}
static void aligned(float target)
{
    in.yaw_deg = target;
    ticks(4); assert(r.phase == ((r.leg & 1U) ? T3_CAPTURE : T3_DIAGONAL));
}
static void capture(void)
{
    line(0, 0, 0); tick(); assert(out.control == T3_HOLD_HEADING);
    in.distance_mm += 10.0f;
    line(2, 1, 1); ticks(4); assert(r.phase == T3_ARC);
    assert(r.leg_start_mm == in.distance_mm);
    tick(); assert(out.control == T3_FOLLOW_LINE);
}
static void arrive_arc(float target)
{
    in.distance_mm = r.leg_start_mm + r.config.arc_mm;
    in.yaw_deg = target; line(0, 0, 0);
    ticks(4); assert(r.phase == (r.leg == 3U ? T3_FINAL_BRAKE : T3_VERTEX_BRAKE));
}
static void lap(uint32_t now, float heading)
{
    init(now, heading);
    float east = heading + r.diagonal_angle_deg;
    arrive_diagonal(); assert(arrivals[T3_POINT_C] == 1U);
    settle(); tick(); assert(out.control == T3_TURN_FORWARD && out.left_mmps == 0.0f && out.right_mmps > 0.0f);
    in.distance_mm += 20.0f; aligned(east); capture();
    arrive_arc(east + 180.0f); assert(arrivals[T3_POINT_B] == 1U);
    settle(); tick(); assert(out.control == T3_TURN_FORWARD && out.left_mmps == 0.0f && out.right_mmps > 0.0f);
    in.distance_mm += 20.0f; aligned(heading + 180.0f + 2.0f * r.diagonal_angle_deg);
    tick(); assert(out.control == T3_HOLD_HEADING && strcmp(CarTask3_LegName(&r), "BD") == 0);
    arrive_diagonal(); assert(arrivals[T3_POINT_D] == 1U);
    settle(); tick(); assert(out.control == T3_TURN_FORWARD && out.left_mmps > 0.0f && out.right_mmps == 0.0f);
    in.distance_mm += 20.0f; aligned(east + 180.0f); capture();
    arrive_arc(east); assert(arrivals[T3_POINT_A] == 0U);
    ticks(21); assert(r.phase == T3_DONE && arrivals[T3_POINT_A] == 1U);
    ticks(50); assert(starts == 1U);
    for (unsigned i=1;i<=4;i++) assert(arrivals[i] == 1U);
    assert(r.elapsed_ms < r.config.deadline_ms);
    CarTask3Config c = CarTask3_DefaultConfig();
    assert(CarTask3_Start(&r, &c, &in)); assert(r.leg == 0U);
}
static void make_arc(void)
{
    init(0U, 0.0f); arrive_diagonal(); settle(); aligned(r.diagonal_angle_deg); capture();
}
static void fault_checks(void)
{
    for (unsigned kind=0;kind<9;kind++) {
        init(0U, 0.0f);
        switch(kind) {
        case 0: in.enabled=0; break;
        case 1: in.stop_requested=1; break;
        case 2: in.protection_fault=1; break;
        case 3: in.heading_ready=0; break;
        case 4: in.sample_age_ms=16U; break;
        case 5: in.yaw_deg=NAN; break;
        case 6: in.distance_mm=INFINITY; break;
        case 7: in.now_ms+=21U; break;
        case 8: in.distance_mm-=6.0f; break;
        }
        tick(); assert(r.phase==T3_FAULT && out.control==T3_HALT && out.signal_point==T3_NO_POINT);
        in.enabled=in.heading_ready=1U; in.stop_requested=in.protection_fault=0U; in.sample_age_ms=0U;
        in.yaw_deg=in.distance_mm=0.0f; tick(); assert(r.phase==T3_FAULT);
    }
    init(0U,0); in.yaw_deg=21.0f; tick(); assert(r.fault==T3_HEADING);
    init(0U,0); in.distance_mm=r.leg_start_mm+r.diagonal_mm+r.config.entry_window_mm+1; tick(); assert(r.fault==T3_EDGE);
    init(0U,0); in.distance_mm=r.leg_start_mm+500; line(2,1,1); ticks(10); assert(r.phase==T3_DIAGONAL && arrivals[T3_POINT_C]==0);
    init(0U,0); in.distance_mm=r.leg_start_mm+r.diagonal_mm; line(2,1,1); ticks(2); line(0,0,0); tick();
    line(2,1,1); ticks(3); assert(r.phase==T3_DIAGONAL); tick(); assert(r.phase==T3_VERTEX_BRAKE);
    in.left_mmps=10.0f; ticks(150); assert(r.fault==T3_SETTLE);
    init(0U,0); arrive_diagonal(); settle(); in.distance_mm=r.phase_start_mm+81; tick(); assert(r.fault==T3_TURN);
    init(0U,0); arrive_diagonal(); settle(); ticks(400); assert(r.fault==T3_TURN);
    init(0U,0); arrive_diagonal(); settle(); aligned(r.diagonal_angle_deg); line(0,0,0);
    in.distance_mm=r.phase_start_mm+61; tick(); assert(r.fault==T3_EDGE);
    init(0U,0); arrive_diagonal(); settle(); aligned(r.diagonal_angle_deg); line(0,0,0);
    ticks(200); assert(r.fault==T3_EDGE);
    make_arc(); line(0,0,0); tick(); assert(out.control==T3_HALT); ticks(10); assert(r.fault==T3_LOST);
    make_arc(); line(4,0,0); ticks(11); assert(r.fault==T3_LOST);
    make_arc(); line(0,0,0); ticks(5); line(2,1,1); tick(); assert(r.phase==T3_ARC && out.control==T3_FOLLOW_LINE);
    make_arc(); in.distance_mm=r.leg_start_mm+r.config.arc_mm; in.yaw_deg=0; line(0,0,0); ticks(11); assert(r.fault==T3_LOST);
    make_arc(); in.distance_mm=r.leg_start_mm+r.config.arc_mm+r.config.exit_window_mm+1; tick(); assert(r.fault==T3_EDGE);
    init(0U,0); ticks(3999); assert(r.fault==T3_TIMEOUT && r.elapsed_ms==40000U);
    init(0U,0); CarTask3Config c=CarTask3_DefaultConfig(); uint32_t start=r.start_ms;
    assert(!CarTask3_Start(&r,&c,&in) && r.start_ms==start); CarTask3_Cancel(&r); tick(); assert(r.fault==T3_ABORT);
    memset(&r,0,sizeof r); c.arc_mm=NAN; assert(!CarTask3_Start(&r,&c,&in) && r.fault==T3_CONFIG);
    memset(&r,0,sizeof r); c=CarTask3_DefaultConfig(); c.deadline_ms=UINT32_MAX;
    assert(!CarTask3_Start(&r,&c,&in) && r.fault==T3_CONFIG);
    memset(&r,0,sizeof r); c=CarTask3_DefaultConfig(); in.left_mmps=6;
    assert(!CarTask3_Start(&r,&c,&in) && r.fault==T3_SETTLE);
}
int main(void)
{
    lap(0U,0.0f); lap(UINT32_MAX-200U,170.0f); lap(0U,-170.0f);
    fault_checks();
    puts("PASS: AC/CB/BD/DA geometry, left/right joins, five unique signals including start, final settled stop, restart, wrap, edge debounce, all fault/timeout guards. Host simulation only.");
    return 0;
}
