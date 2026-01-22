#define _GNU_SOURCE
#include "collision_response_real.h"

#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>

// CAN 인터페이스
#include "can_interface_real.h"

// [수정] 부저 제어용 핀 번호 (daemon2.c와 동일하게 맞춤)
// 실제 제어는 daemon2.c의 g_handlers.on_severity 콜백에서 처리하는 것이 구조상 더 깔끔하지만,
// 여기서 직접 제어하려면 GPIO 파일 입출력을 써야 합니다.
// (가장 좋은 방법은 daemon2.c의 콜백 함수를 통해 부저를 울리는 것입니다.)

// ----------------- 내부 시간 유틸 -----------------
static uint32_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

// 0~359도에서 각도 차이(최소 각거리) 계산
static uint16_t ang_diff_deg(uint16_t a, uint16_t b)
{
    uint16_t d = (a > b) ? (a - b) : (b - a);
    return (d > 180) ? (360 - d) : d;
}

// ----------------- Door Unlock (Stub) -----------------
// 현재는 로그만 출력 (릴레이 연결 시 GPIO 제어 추가 가능)
static void door_unlock_pulse(uint32_t pulse_ms, int enable_stdout)
{
    if (enable_stdout) {
        printf("[CRESP] Door Unlock Pulse (%u ms)\n", (unsigned)pulse_ms);
        fflush(stdout);
    }
    // TODO: GPIO 제어가 필요하다면 여기에 추가 (sysfs 방식)
    (void)pulse_ms;
}

// ----------------- 상태/스레드 -----------------
static pthread_t g_thr;
static pthread_mutex_t g_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cv  = PTHREAD_COND_INITIALIZER;

static int g_running = 0;

static cresp_config_t g_cfg = {
    .heading_swing_threshold_deg = 30,
    .crash_cooldown_ms = 1000,
    .door_unlock_pulse_ms = 300,
    .enable_stdout = 1,
};

static cresp_handlers_t g_h = {0};

// 최근 heading 값
static uint16_t g_heading_deg = 0;
static uint32_t g_heading_ts  = 0;

// 충돌 센서 시퀀스
static uint32_t g_collision_seq = 0;
static uint32_t g_seen_collision_seq = 0;

// 충돌 처리 중복 방지 타이머
static uint32_t g_last_crash_ms = 0;

// 충돌 시점의 기준 heading 및 흔들림 추적
static uint16_t g_crash_heading_base = 0;
static uint32_t g_crash_window_start_ms = 0;
static uint16_t g_crash_max_swing_deg = 0;
static int      g_crash_window_active = 0;
static int      g_sev3_sent = 0;

// Severity 전달 (콜백 호출)
// [중요] 부저 울리는 역할은 여기서 호출되는 콜백(daemon2.c)이 담당합니다.
static void emit_severity(uint8_t sev)
{
    if (g_cfg.enable_stdout) {
        printf("[CRESP] Severity Emit: %u\n", (unsigned)sev);
        fflush(stdout);
    }
    if (g_h.on_severity) g_h.on_severity(sev);
}

// CAN 브레이크 3단계 전송 (BRAKE_YIELD)
static void send_brake_stage3(void)
{
    uint16_t t16 = (uint16_t)(now_ms() & 0xFFFFu);
    (void)CANIF_send_brakelight(BRAKE_YIELD, t16);
}

static void* worker(void* arg)
{
    (void)arg;

    pthread_mutex_lock(&g_mtx);
    while (g_running) {
        // 1) 이벤트 대기
        while (g_running && g_collision_seq == g_seen_collision_seq) {
            pthread_cond_wait(&g_cv, &g_mtx);
        }
        if (!g_running) break;

        // 이벤트 처리 시작
        g_seen_collision_seq = g_collision_seq;
        uint32_t t_now = now_ms();

        // 쿨다운 체크
        if (g_last_crash_ms && (t_now - g_last_crash_ms) < g_cfg.crash_cooldown_ms) {
            if (g_cfg.enable_stdout) {
                printf("[CRESP] Collision Ignored (Cooldown)\n");
                fflush(stdout);
            }
            continue;
        }
        g_last_crash_ms = t_now;

        // 충돌 윈도우 초기화
        g_crash_heading_base = g_heading_deg;
        g_crash_window_start_ms = t_now;
        g_crash_max_swing_deg = 0;
        g_crash_window_active = 1;
        g_sev3_sent = 0;

        pthread_mutex_unlock(&g_mtx);

        // --- [Action Sequence] ---

        // 1) 도어 언락
        door_unlock_pulse(g_cfg.door_unlock_pulse_ms, g_cfg.enable_stdout);

        // 2) 브레이크등 (CAN)
        send_brake_stage3();

        // 3) 사고 규모 2 (심각) 전파 -> 여기서 부저가 울려야 함!
        emit_severity(2);

        // -------------------------

        pthread_mutex_lock(&g_mtx);

        // 1초 동안 흔들림 감지
        while (g_running && g_crash_window_active) {
            uint32_t elapsed = now_ms() - g_crash_window_start_ms;
            
            // 1초 종료 시점
            if (elapsed >= 1000u) {
                // 흔들림이 컸으면 Severity 3 (매우 심각 - 전복 등) 전송
                if (!g_sev3_sent && g_crash_max_swing_deg >= g_cfg.heading_swing_threshold_deg) {
                    pthread_mutex_unlock(&g_mtx);
                    emit_severity(3); 
                    pthread_mutex_lock(&g_mtx);
                    g_sev3_sent = 1;
                }
                g_crash_window_active = 0;
                break;
            }

            // 남은 시간만큼 대기 (Timed Wait)
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            uint32_t remain_ms = 1000u - elapsed;
            ts.tv_sec  += remain_ms / 1000u;
            ts.tv_nsec += (long)(remain_ms % 1000u) * 1000000L;
            if (ts.tv_nsec >= 1000000000L) {
                ts.tv_sec += 1;
                ts.tv_nsec -= 1000000000L;
            }
            pthread_cond_timedwait(&g_cv, &g_mtx, &ts);
        }
    }
    pthread_mutex_unlock(&g_mtx);
    return NULL;
}

// ----------------- Public API -----------------
int CRESP_init(const cresp_config_t* cfg, const cresp_handlers_t* h)
{
    pthread_mutex_lock(&g_mtx);

    if (cfg) g_cfg = *cfg;
    if (h)   g_h   = *h;

    g_heading_deg = 0;
    g_heading_ts  = 0;
    g_collision_seq = 0;
    g_seen_collision_seq = 0;
    g_last_crash_ms = 0;

    pthread_mutex_unlock(&g_mtx);
    return 0;
}

int CRESP_start(void)
{
    pthread_mutex_lock(&g_mtx);
    if (g_running) { pthread_mutex_unlock(&g_mtx); return 0; }
    g_running = 1;
    pthread_mutex_unlock(&g_mtx);

    if (pthread_create(&g_thr, NULL, worker, NULL) != 0) {
        // pthread_mutex_lock(&g_mtx);
        g_running = 0;
        // pthread_mutex_unlock(&g_mtx);
        return -1;
    }
    return 0;
}

void CRESP_stop(void)
{
    pthread_mutex_lock(&g_mtx);
    if (!g_running) { pthread_mutex_unlock(&g_mtx); return; }
    g_running = 0;
    pthread_cond_broadcast(&g_cv);
    pthread_mutex_unlock(&g_mtx);

    pthread_join(g_thr, NULL);
}

void CRESP_on_collision_sensor(void)
{
    pthread_mutex_lock(&g_mtx);
    g_collision_seq++;
    pthread_cond_signal(&g_cv);
    pthread_mutex_unlock(&g_mtx);

    if (g_cfg.enable_stdout) {
        // printf("[CRESP] Collision Sensor Event!\n");
    }
}

void CRESP_on_heading_deg(uint16_t heading_deg)
{
    if (heading_deg >= 360) heading_deg %= 360;

    pthread_mutex_lock(&g_mtx);
    g_heading_deg = heading_deg;
    g_heading_ts  = now_ms();

    if (g_crash_window_active) {
        uint16_t swing = ang_diff_deg(g_crash_heading_base, g_heading_deg);
        if (swing > g_crash_max_swing_deg) g_crash_max_swing_deg = swing;
    }

    pthread_cond_signal(&g_cv);
    pthread_mutex_unlock(&g_mtx);
}