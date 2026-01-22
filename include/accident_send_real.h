#ifndef ACCIDENT_SEND_MODULE_H
#define ACCIDENT_SEND_MODULE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================
//  Protocol Definition (WL-2)
// =============================================================

// [WL-2 Packet] 6바이트 사고 요약 패킷 구조
// Byte0-1: Distance(12bit, m) << 4 | RSV(4bit)
// Byte2  : Lane (1/2/3)
// Byte3  : Severity(4bit) << 4 | RSV(4bit)
// Byte4-5: Timestamp (uint16_t, 10ms tick)
typedef struct __attribute__((packed)) {
    uint16_t dist_rsv;   // [15:4]=distance(m, 0..4095), [3:0]=rsv
    uint8_t  lane;       // 1/2/3
    uint8_t  sev_rsv;    // [7:4]=severity(0..15), [3:0]=rsv
    uint16_t ts_10ms;    // 10ms tick (Overflow 허용)
} wl2_packet_t;

// [사고 유형]
typedef enum {
    ACC_TYPE_UNKNOWN    = 0,
    ACC_TYPE_COLLISION  = 1,
    ACC_TYPE_HARD_BRAKE = 2,
    ACC_TYPE_OBSTACLE   = 3,
} acc_type_t;

// =============================================================
//  Callback & Provider Types
// =============================================================

// [데이터 제공자] 현재 주행 정보를 조회하기 위한 함수 포인터들
// (driving_info_module 등의 데이터를 당겨오기 위함)
typedef struct {
    // 현재 차선 (1/2/3), 모르면 0
    uint8_t  (*get_lane)(void);

    // 전방 거리(m). 없으면 0
    uint16_t (*get_front_distance_m)(void);

    // 객체 인식 요약 정보 (8bit), 없으면 0
    uint8_t  (*get_object_hint)(void);
} accinfo_provider_t;

// [전송 함수] 만들어진 패킷을 외부로 전송하는 함수 포인터 (UART, SPI 등)
// buf: 전송할 데이터, len: 길이
typedef int (*comm_send_fn)(const uint8_t* buf, uint32_t len);

// =============================================================
//  Configuration & Events
// =============================================================

// [설정] 모듈 초기화 파라미터
typedef struct {
    // 같은 종류의 사고 정보 전송 최소 간격 (ms) - 폭주 방지
    uint32_t min_send_interval_ms;

    // 전방 거리를 모를 때 사용할 기본값(m)
    uint16_t default_distance_m;

    // 차선을 모를 때 사용할 기본값
    uint8_t  default_lane;

    // 로그 출력 여부 (1:ON)
    int enable_stdout;
} accsend_config_t;

// [입력 이벤트] 다른 모듈(Risk Module 등)에서 전달하는 사고 정보
typedef struct {
    uint8_t     severity;   // 심각도 (1:주의, 2:경고, 3:위험)
    acc_type_t  type;       // 사고 유형
    uint8_t     action;     // 추가 액션 정보
    uint16_t    distance_m; // 0이면 Provider 값 사용
    uint8_t     lane;       // 0이면 Provider 값 사용
} acc_event_t;

// =============================================================
//  Public API Functions
// =============================================================

// 모듈 초기화
// send_fn: UARTIF_send_raw_bytes 등을 연결
int  ACCSEND_init(const accsend_config_t* cfg,
                  const accinfo_provider_t* provider,
                  comm_send_fn send_fn);

// 모듈 시작 (전송 스레드 생성)
int  ACCSEND_start(void);

// 모듈 정지
void ACCSEND_stop(void);

// [핵심] 사고 발생 이벤트 접수 (Thread-Safe, Non-blocking)
// 큐에 넣고 즉시 리턴됨. 실제 전송은 스레드가 처리.
int  ACCSEND_on_event(const acc_event_t* ev);

#ifdef __cplusplus
}
#endif

#endif // ACCIDENT_SEND_MODULE_H