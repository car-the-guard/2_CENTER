#ifndef CAN_INTERFACE_MODULE_H
#define CAN_INTERFACE_MODULE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================
//  CAN ID Definition (11-bit Standard)
// =============================================================
enum {
    CANID_VIBRATION     = 0x08,  // [RX] 충돌 감지 (Data[0]=0xFF)
    CANID_AEB_CTRL      = 0x10,  // [TX] AEB 제어 (Data[0]=0xFF:On, 0x0F:Off)
    CANID_MOTOR_CTRL    = 0x12,  // [TX] 모터 제어 (4채널)
    CANID_ULTRASONIC    = 0x24,  // [RX] 초음파 센서 (Data 2Byte = cm * 10)
    CANID_SPEED         = 0x28,  // [RX] 속도 (Data 4Byte = 0.0001 m/s)
    CANID_ACCEL         = 0x2C,  // [RX] 가속도 (Raw 2Byte + Filt 2Byte)
    CANID_MAGNETIC      = 0x30,  // [RX] 지자기 (Data 2Byte = 0~359 deg)
    CANID_BRAKELIGHT    = 0x48,  // [TX] 브레이크등 (0:Off, 1:On, 2:Blink, 3:Yield)
    CANID_SEATBELT_CTRL = 0x49   // [TX] 안전벨트 프리텐셔너 (Data[0]=1:Lock)
};

// =============================================================
//  Data Structures
// =============================================================

// [공통] 8바이트 페이로드 구조 (STM32와 약속된 프로토콜)
typedef struct __attribute__((packed)) {
    uint32_t data;      // byte0~3 (핵심 데이터)
    uint16_t time_ms;   // byte4~5 (타임스탬프)
    uint8_t  reserved;  // byte6
    uint8_t  crc8;      // byte7   (무결성 검사)
} canif_payload_t;

// [모터] 4바이트에 4개 모터 제어값 매핑
typedef struct {
    int8_t ch[4];   // -100 ~ 100 권장 (0=정지)
} canif_motor4_t;

// [브레이크등] 동작 모드
typedef enum {
    BRAKE_OFF   = 0,
    BRAKE_ON    = 1,
    BRAKE_BLINK = 2,
    BRAKE_YIELD = 3,  // 양보/비상등 (화살표 모드 등)
} canif_brake_mode_t;

// =============================================================
//  Configuration & Handlers
// =============================================================

// [설정] 초기화 파라미터
typedef struct {
    const char* ifname;      // 인터페이스 이름 (예: "can0")
    uint8_t     motor_map[4];// 모터 채널 매핑 (예: {0,1,2,3})
} canif_config_t;

// [콜백 함수 타입]
typedef void (*canif_on_u16_fn)(uint16_t v);
typedef void (*canif_on_i16x2_fn)(int16_t raw, int16_t filt);
typedef void (*canif_on_u32_fn)(uint32_t v);
typedef void (*canif_on_void_fn)(void);

// [수신 핸들러] CAN 데이터 수신 시 호출될 함수들
typedef struct {
    canif_on_u16_fn   on_ultrasonic_cm; // 초음파 거리 (mm 단위로 변환되어 전달됨)
    canif_on_u32_fn   on_speed_q4;      // 속도 (0.0001 m/s 단위)
    canif_on_i16x2_fn on_accel_x;       // 가속도 X축
    canif_on_u16_fn   on_heading_deg;   // 지자기 (0~359)
    canif_on_void_fn  on_vibration;     // 충돌 센서 감지
} canif_rx_handlers_t;

// =============================================================
//  Public API Functions
// =============================================================

// 모듈 초기화 및 CAN 소켓 오픈
int  CANIF_init(const canif_config_t* cfg, const canif_rx_handlers_t* rxh);

// 수신 스레드 시작
int  CANIF_start(void);

// 수신 스레드 종료 및 소켓 닫기
void CANIF_stop(void);

// [TX] AEB (긴급 제동) 활성화 (모터 Lock)
int CANIF_send_aeb_enable(uint16_t time_ms);

// [TX] AEB 해제 (모터 Release)
int CANIF_send_aeb_disable(uint16_t time_ms);

// [TX] 브레이크등 제어
int CANIF_send_brakelight(canif_brake_mode_t mode, uint16_t time_ms);

// [TX] 안전벨트 프리텐셔너 제어 (1:당김, 0:풀림)
int CANIF_send_seatbelt(int enable);

// [TX] 모터 4채널 제어
int CANIF_send_motor4(const canif_motor4_t* m, uint16_t time_ms);

// [TX] 디버그용 Raw 데이터 전송
int CANIF_send_raw(uint16_t canid, const canif_payload_t* p);

#ifdef __cplusplus
}
#endif

#endif // CAN_INTERFACE_MODULE_H