#define _GNU_SOURCE
#include "driving_info_real.h"
#include <pthread.h>
#include <string.h>
#include <time.h>

static pthread_rwlock_t g_rw = PTHREAD_RWLOCK_INITIALIZER;
static int g_inited = 0;
static dim_snapshot_t g_s;
static double g_lat = 0.0;
static double g_lon = 0.0;

static uint32_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

static uint16_t norm_heading(uint16_t deg)
{
    return (uint16_t)(deg % 360u);
}

int DIM_init(void)
{
    if (g_inited) return 0;

    pthread_rwlock_init(&g_rw, NULL);

    pthread_rwlock_wrlock(&g_rw);
    memset(&g_s, 0, sizeof(g_s));
    g_s.lane = DIM_LANE_UNKNOWN;
    g_s.front_obj.type = DIM_OBJ_NONE;
    g_s.front_obj.confidence_pct = 0;
    g_s.front_obj.rel_lane = 0;
    g_s.front_obj.dist_cm = 0;

    g_s.ts_lane_ms = 0;
    g_s.ts_obj_ms = 0;
    g_s.ts_speed_ms = 0;
    g_s.ts_ultra_ms = 0;
    g_s.ts_heading_ms = 0;
    g_s.seq = 0;
    pthread_rwlock_unlock(&g_rw);

    g_inited = 1;
    return 0;
}

void DIM_deinit(void)
{
    if (!g_inited) return;
    pthread_rwlock_destroy(&g_rw);
    g_inited = 0;
}

void DIM_update_lane(dim_lane_t lane)
{
    uint32_t t = now_ms();
    pthread_rwlock_wrlock(&g_rw);
    g_s.lane = lane;
    g_s.ts_lane_ms = t;
    g_s.seq++;
    pthread_rwlock_unlock(&g_rw);
}

void DIM_update_front_object(const dim_front_object_t* obj)
{
    if (!obj) return;
    uint32_t t = now_ms();
    pthread_rwlock_wrlock(&g_rw);
    g_s.front_obj = *obj;
    g_s.ts_obj_ms = t;
    g_s.seq++;
    pthread_rwlock_unlock(&g_rw);
}

void DIM_update_speed(uint32_t speed_1e5_mps)
{
    uint32_t t = now_ms();
    pthread_rwlock_wrlock(&g_rw);
    g_s.car_speed_1e5_mps = speed_1e5_mps;
    g_s.ts_speed_ms = t;
    g_s.seq++;
    pthread_rwlock_unlock(&g_rw);
}

void DIM_update_ultrasonic(uint16_t dist_mm)
{
    uint32_t t = now_ms();
    pthread_rwlock_wrlock(&g_rw);
    g_s.ultra_dist_mm = dist_mm;
    g_s.ts_ultra_ms = t;
    g_s.seq++;
    pthread_rwlock_unlock(&g_rw);
}

void DIM_update_heading(uint16_t heading_deg)
{
    uint32_t t = now_ms();
    pthread_rwlock_wrlock(&g_rw);
    g_s.heading_deg = norm_heading(heading_deg);
    g_s.ts_heading_ms = t;
    g_s.seq++;
    pthread_rwlock_unlock(&g_rw);
}

int DIM_get_snapshot(dim_snapshot_t* out)
{
    if (!out) return -1;
    pthread_rwlock_rdlock(&g_rw);
    *out = g_s;
    pthread_rwlock_unlock(&g_rw);

    out->gps_lat = g_lat;
    out->gps_lon = g_lon;
    return 0;
}

dim_lane_t DIM_get_lane(void)
{
    pthread_rwlock_rdlock(&g_rw);
    dim_lane_t v = g_s.lane;
    pthread_rwlock_unlock(&g_rw);
    return v;
}

uint32_t DIM_get_speed(void)
{
    pthread_rwlock_rdlock(&g_rw);
    uint32_t v = g_s.car_speed_1e5_mps;
    pthread_rwlock_unlock(&g_rw);
    return v;
}

uint16_t DIM_get_ultrasonic(void)
{
    pthread_rwlock_rdlock(&g_rw);
    uint16_t v = g_s.ultra_dist_mm;
    pthread_rwlock_unlock(&g_rw);
    return v;
}

uint16_t DIM_get_heading(void)
{
    pthread_rwlock_rdlock(&g_rw);
    uint16_t v = g_s.heading_deg;
    pthread_rwlock_unlock(&g_rw);
    return v;
}

int DIM_get_front_object(dim_front_object_t* out)
{
    if (!out) return -1;
    pthread_rwlock_rdlock(&g_rw);
    *out = g_s.front_obj;
    pthread_rwlock_unlock(&g_rw);
    return 0;
}

static int is_fresh(uint32_t ts_ms, uint32_t max_age_ms)
{
    if (ts_ms == 0) return 0;
    uint32_t t = now_ms();
    return (t - ts_ms) <= max_age_ms;
}

int DIM_is_fresh_lane(uint32_t max_age_ms)
{
    pthread_rwlock_rdlock(&g_rw);
    uint32_t ts = g_s.ts_lane_ms;
    pthread_rwlock_unlock(&g_rw);
    return is_fresh(ts, max_age_ms);
}

int DIM_is_fresh_speed(uint32_t max_age_ms)
{
    pthread_rwlock_rdlock(&g_rw);
    uint32_t ts = g_s.ts_speed_ms;
    pthread_rwlock_unlock(&g_rw);
    return is_fresh(ts, max_age_ms);
}

int DIM_is_fresh_ultra(uint32_t max_age_ms)
{
    pthread_rwlock_rdlock(&g_rw);
    uint32_t ts = g_s.ts_ultra_ms;
    pthread_rwlock_unlock(&g_rw);
    return is_fresh(ts, max_age_ms);
}

int DIM_is_fresh_heading(uint32_t max_age_ms)
{
    pthread_rwlock_rdlock(&g_rw);
    uint32_t ts = g_s.ts_heading_ms;
    pthread_rwlock_unlock(&g_rw);
    return is_fresh(ts, max_age_ms);
}

int DIM_is_fresh_obj(uint32_t max_age_ms)
{
    pthread_rwlock_rdlock(&g_rw);
    uint32_t ts = g_s.ts_obj_ms;
    pthread_rwlock_unlock(&g_rw);
    return is_fresh(ts, max_age_ms);
}

// [추가] GPS 업데이트 구현
void DIM_update_gps(double lat, double lon)
{
    g_lat = lat;
    g_lon = lon;
}

// [추가] AI 데이터 수신용 구현 (daemon2.c에서 호출)
void DIM_update_front_obj(uint8_t type, uint16_t dist_cm)
{
    uint32_t t = now_ms();
    pthread_rwlock_wrlock(&g_rw);

    // 타입 매핑
    dim_obj_type_t obj_type = DIM_OBJ_NONE;
    if (type == 1) obj_type = DIM_OBJ_OBSTACLE;
    else if (type == 2) obj_type = DIM_OBJ_CONE;
    else obj_type = DIM_OBJ_UNKNOWN;

    // 객체 정보 업데이트
    g_s.front_obj.type = obj_type;
    g_s.front_obj.dist_cm = dist_cm;
    g_s.front_obj.confidence_pct = 80; // AI 수신 데이터이므로 신뢰도 높음 가정
    g_s.front_obj.rel_lane = 0;        // 일단 내 차선에 있다고 가정

    g_s.ts_obj_ms = t;
    g_s.seq++;
    
    pthread_rwlock_unlock(&g_rw);
}