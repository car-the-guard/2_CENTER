#ifndef COLLISION_RESPONSE_MODULE_H
#define COLLISION_RESPONSE_MODULE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================
//  Data Types
// =============================================================

// [설정] 모듈 동작 파라미터
typedef struct {
    // 충돌 후 1초 동안 "방향 흔들림" 판정 임계값 (단위: 도)
    // 이 값보다 크게 흔들리면 전복(Severity 3)으로 판단
    uint16_t heading_swing_threshold_deg;   // 예: 30

    // 충돌 이벤트 디바운스/쿨다운 (단위: ms)
    // 연속적인 충돌 신호 무시 시간
    uint32_t crash_cooldown_ms;             // 예: 1000

    // 도어 잠금 해제 신호 유지 시간 (단위: ms)
    // (현재는 로그 출력용, 추후 GPIO/릴레이 제어 연동)
    uint32_t door_unlock_pulse_ms;          // 예: 300

    // 표준 출력(stdout) 로그 활성화 여부 (1: ON, 0: OFF)
    int enable_stdout;
} cresp_config_t;

// [콜백] 사고 심각도 전파 함수 타입
// severity: 2(충돌 감지), 3(전복/대형 사고)
typedef void (*cresp_on_severity_fn)(uint8_t severity);

// [핸들러] 외부로 이벤트를 전달할 핸들러 구조체
typedef struct {
    cresp_on_severity_fn on_severity; // 필수: daemon2.c의 부저/통신 로직과 연결됨
} cresp_handlers_t;

// =============================================================
//  Public API Functions
// =============================================================

// 모듈 초기화
int  CRESP_init(const cresp_config_t* cfg, const cresp_handlers_t* h);

// 모듈 시작 (스레드 생성)
int  CRESP_start(void);

// 모듈 정지 (스레드 종료)
void CRESP_stop(void);

// [입력 1] 충돌 감지 센서 인터럽트 발생 시 호출
// 물리적 충돌(G센서, 충격 센서 등)이 감지되었을 때 호출
void CRESP_on_collision_sensor(void);

// [입력 2] 지자기(Heading) 데이터 업데이트
// 차량의 방향 정보를 업데이트하여 충돌 후 회전/전복 여부를 판단
// driving_info_module이나 CAN 인터페이스에서 호출
void CRESP_on_heading_deg(uint16_t heading_deg);

#ifdef __cplusplus
}
#endif

#endif // COLLISION_RESPONSE_MODULE_H