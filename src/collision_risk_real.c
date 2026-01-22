#include "collision_risk_real.h"
#include <stdio.h>
#include <math.h>

// 설정값
#define TTC_WARN_SEC        3.0f   // 경고
#define TTC_WARN_RESET_SEC  4.0f   // 경고 해제
#define TTC_AEB_SEC         2.3f   // AEB 작동

#define SPEED_PRESAFE_MPS   0.2f   // 안전벨트 진입 속도
#define SPEED_STOP_MPS      0.1f   // 정지 판단 기준 (노이즈 대비 완화)
#define SPEED_RESUME_MPS    0.15f  // 재출발 판단 속도

#define ROLLOVER_DEG        40.0f  
#define WAIT_STOP_MS        3000   
#define WAIT_IMU_MS         300    

static crm_config_t g_cfg;
static crm_callbacks_t g_cb;
static int g_is_running = 0;

static crm_state_t g_state = CRM_STATE_NORMAL;
static uint32_t g_state_entry_time = 0;

static void set_state(crm_state_t next_state, uint32_t now) {
    if (g_state != next_state) {
        printf("[CRM] State Change: %d -> %d (Time: %u)\n", g_state, next_state, now);
        g_state = next_state;
        g_state_entry_time = now;
    }
}

void CRM_init(const crm_config_t* cfg, const crm_callbacks_t* cb) {
    if (cfg) g_cfg = *cfg;
    if (cb) g_cb = *cb;
}

void CRM_start(void) { g_is_running = 1; }
void CRM_stop(void) { g_is_running = 0; }
crm_state_t CRM_get_current_state(void) { return g_state; }

void CRM_update_inputs(const crm_inputs_t* in) {
    if (!g_is_running) return;

    uint32_t now = in->timestamp_ms;
    float dist = in->dist_m;
    float speed = in->rel_speed_mps;

    // TTC 계산 (나눗셈 0 방지)
    float ttc = 999.0f;
    if (speed > 0.1f && dist > 0.001f) {
        ttc = dist / speed;
    }

    // [수정] 로그 폭탄 방지를 위해 주석 처리 (필요할 때만 주석 해제)
    printf("초음파 거리 : %.2f m, 속도 : %.2f m/s, TTC: %.2f s\n", dist, speed, ttc);

    uint32_t elapsed = now - g_state_entry_time;

    switch (g_state) {
        // ---------------------------------------------------------
        // 1. 정상 주행
        // ---------------------------------------------------------
        case CRM_STATE_NORMAL:
            if (g_cb.send_brake_lamp_level) g_cb.send_brake_lamp_level(0);
            if (g_cb.send_aeb) g_cb.send_aeb(0);
            if (g_cb.control_seatbelt) g_cb.control_seatbelt(0);

            if (ttc <= TTC_WARN_SEC) {
                set_state(CRM_STATE_WARNING, now);
            }
            break;

        // ---------------------------------------------------------
        // 2. 경고 단계 (브레이크등 1단계)
        // ---------------------------------------------------------
        case CRM_STATE_WARNING:
            if (g_cb.send_brake_lamp_level) g_cb.send_brake_lamp_level(1);

            // TTC가 더 짧아지면 AEB 단계로
            if (ttc <= TTC_AEB_SEC) {
                set_state(CRM_STATE_AEB_ACTIVATED, now);
            }
            // 상황 해제
            else if (ttc >= TTC_WARN_RESET_SEC && elapsed >= 1000) {
                set_state(CRM_STATE_NORMAL, now);
            }
            break;

        // ---------------------------------------------------------
        // 3. AEB 작동 (브레이크등 2, 사고 1)
        // ---------------------------------------------------------
        case CRM_STATE_AEB_ACTIVATED:
            if (g_cb.send_aeb) g_cb.send_aeb(1); // 모터 Lock
            if (g_cb.send_brake_lamp_level) g_cb.send_brake_lamp_level(2);
            if (g_cb.notify_accident_scale) g_cb.notify_accident_scale(1);
            if (g_cb.control_seatbelt) g_cb.control_seatbelt(1);

            // 속도가 너무 빠르면 프리세이프 진입
            if (speed >= SPEED_PRESAFE_MPS) {
                set_state(CRM_STATE_PRE_SAFE, now);
            }
            // 충돌했거나 멈췄으면 대기 단계로
            else if (in->is_collision || speed <= SPEED_STOP_MPS) {
                set_state(CRM_STATE_POST_CRASH_WAIT, now);
            }
            break;

        // ---------------------------------------------------------
        // 4. 프리세이프 (안전벨트 꽉)
        // ---------------------------------------------------------
        case CRM_STATE_PRE_SAFE:
            if (g_cb.control_seatbelt) g_cb.control_seatbelt(1);
            
            if (in->is_collision || speed <= SPEED_STOP_MPS) {
                set_state(CRM_STATE_POST_CRASH_WAIT, now);
            }
            break;

        // ---------------------------------------------------------
        // 5. 사고 직후/급정거 후 대기 (브레이크등 3, 사고 2)
        // ---------------------------------------------------------
        case CRM_STATE_POST_CRASH_WAIT:
            if (g_cb.send_brake_lamp_level) g_cb.send_brake_lamp_level(3);
            if (g_cb.notify_accident_scale) g_cb.notify_accident_scale(2); // 부저 울림
            
            // [복귀 편의성 로직]
            // 손을 치워서 거리가 확보되거나(1.2m), TTC가 안전해지면
            // 상태는 아직 Wait이지만 AEB Lock(모터 잠금)은 미리 풀어줍니다.
            // 그래야 사용자가 앱으로 차를 뒤로 뺄 수 있습니다.
            if (dist >= 1.2f || (speed > 0.1f && ttc >= 4.0f)) {
                 if (g_cb.send_aeb) g_cb.send_aeb(0); // AEB 해제 (운전 가능)
            } else {
                 if (g_cb.send_aeb) g_cb.send_aeb(1); // AEB 유지
            }

            // 전복 감지
            if (elapsed >= WAIT_IMU_MS) {
                if (fabsf(in->roll_deg) >= ROLLOVER_DEG) {
                    set_state(CRM_STATE_ROLLOVER, now);
                    return;
                }
            }

            // A. 실제 충돌 센서 감지
            if (in->is_collision) {
                set_state(CRM_STATE_ACCIDENT_CONFIRM, now);
            }
            // B. 3초 경과 (충돌 없이 멈춤) -> 사용자 조작 시 복귀
            else if (elapsed >= WAIT_STOP_MS) {
                if (speed >= SPEED_RESUME_MPS) {
                    set_state(CRM_STATE_RECOVERY, now);
                }
            }
            break;

        // ---------------------------------------------------------
        // 6. 사고 확정 (에어백 전개)
        // ---------------------------------------------------------
        case CRM_STATE_ACCIDENT_CONFIRM:
            if (g_cb.deploy_airbag) g_cb.deploy_airbag(1);
            if (g_cb.notify_accident_scale) g_cb.notify_accident_scale(3); // 심각한 경고음
            if (g_cb.send_brake_lamp_level) g_cb.send_brake_lamp_level(3);

            if (fabsf(in->roll_deg) >= ROLLOVER_DEG) {
                set_state(CRM_STATE_ROLLOVER, now);
            }
            break;

        // ---------------------------------------------------------
        // 7. 전복
        // ---------------------------------------------------------
        case CRM_STATE_ROLLOVER:
            if (g_cb.notify_accident_scale) g_cb.notify_accident_scale(3);
            if (g_cb.send_brake_lamp_level) g_cb.send_brake_lamp_level(3);
            // printf("[CRM] ROLLOVER!\n"); // 중요 로그는 남김
            break;

        // ---------------------------------------------------------
        // 8. 수습 및 복귀
        // ---------------------------------------------------------
        case CRM_STATE_RECOVERY:
            // 모든 경고 해제 및 초기화
            if (g_cb.send_aeb) g_cb.send_aeb(0);
            if (g_cb.send_brake_lamp_level) g_cb.send_brake_lamp_level(0);
            if (g_cb.control_seatbelt) g_cb.control_seatbelt(0);

            set_state(CRM_STATE_NORMAL, now);
            break;
    }
}