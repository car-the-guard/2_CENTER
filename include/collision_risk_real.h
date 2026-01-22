#ifndef COLLISION_RISK_MODULE_H
#define COLLISION_RISK_MODULE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================
//  Data Types
// =============================================================

// [상태 정의] 충돌 위험 판단 상태 머신 (FSM)
typedef enum {
    CRM_STATE_NORMAL = 0,           // 정상 주행
    CRM_STATE_WARNING,              // 경고 (브레이크등 1단계)
    CRM_STATE_AEB_ACTIVATED,        // AEB 작동 (브레이크등 2단계, 사고 1단계)
    CRM_STATE_PRE_SAFE,             // 프리세이프 (안전벨트 당김)
    CRM_STATE_POST_CRASH_WAIT,      // 충돌/정지 직후 판단 대기 (3초)
    CRM_STATE_ACCIDENT_CONFIRM,     // 사고 확정 (에어백 전개, 사고 3단계)
    CRM_STATE_ROLLOVER,             // 전복 감지 (매우 심각)
    CRM_STATE_RECOVERY              // 상황 종료 및 복귀
} crm_state_t;

// [설정] 모듈 동작 파라미터
typedef struct {
    float period_ms;        // 주기 (예: 50ms)
    float max_data_age_ms;  // 데이터 유효 시간
} crm_config_t;

// [입력] 판단에 필요한 센서 데이터 (Snapshot)
typedef struct {
    float dist_m;           // 전방 거리 (m)
    float rel_speed_mps;    // 상대 속도 (m/s, +는 접근 중)
    float roll_deg;         // 차량 기울기 (전복 감지용)
    int   is_collision;     // 물리적 충돌 센서 감지 여부 (1:충돌)
    int   obj_type;         // 객체 종류 (장애물, 라바콘 등)
    uint32_t timestamp_ms;  // 데이터 생성 시간
} crm_inputs_t;

// [콜백] 판단 결과에 따른 액션 실행 함수들
typedef struct {
    // 브레이크등 제어 (0:OFF, 1:Warn, 2:AEB, 3:Stop/Emerg)
    void (*send_brake_lamp_level)(int level);
    
    // AEB 모터 제어 (1:Lock/Brake, 0:Release)
    void (*send_aeb)(int enable);
    
    // 사고 상황 전파 (1:주의, 2:위험, 3:심각) -> 부저/통신
    void (*notify_accident_scale)(int scale);
    
    // 에어백 전개 (1:Deploy)
    void (*deploy_airbag)(int enable);
    
    // 안전벨트 제어 (1:Tension, 0:Release)
    void (*control_seatbelt)(int enable);
} crm_callbacks_t;

// =============================================================
//  Public API Functions
// =============================================================

// 모듈 초기화
void CRM_init(const crm_config_t* cfg, const crm_callbacks_t* cb);

// 모듈 시작/정지
void CRM_start(void);
void CRM_stop(void);

// [핵심] 센서 데이터 업데이트 및 판단 로직 실행 (주기적으로 호출)
void CRM_update_inputs(const crm_inputs_t* in);

// 현재 상태 조회
crm_state_t CRM_get_current_state(void);

#ifdef __cplusplus
}
#endif

#endif // COLLISION_RISK_MODULE_H