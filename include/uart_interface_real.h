#ifndef UART_INTERFACE_H
#define UART_INTERFACE_H

#include <stdint.h>

// [설정] 초기화 구조체
typedef struct {
    const char* dev_path;  // 장치 경로 (예: /dev/ttyAMA1)
    int baudrate;          // 통신 속도 (예: 9600)
} uartif_config_t;

// [수신] 데이터를 처리할 콜백 함수 타입
// data: 수신된 Raw 데이터 포인터
// len: 수신된 데이터 길이
typedef void (*uart_on_external_data_fn)(uint8_t* data, uint16_t len);

// [핸들러] 콜백 등록 구조체
typedef struct {
    uart_on_external_data_fn on_data; // 데이터 수신 시 호출될 함수
} uartif_rx_handlers_t;

// =============================================================
//  Public API Functions
// =============================================================

// 초기화 및 UART 포트 오픈
int  UARTIF_init(const uartif_config_t* cfg, const uartif_rx_handlers_t* handlers);

// 수신 스레드 시작
int  UARTIF_start(void);

// 수신 스레드 종료 및 포트 닫기
void UARTIF_stop(void);

// [송신] 내 사고 정보를 전송 (WL-3 프로토콜)
// accident_type: 사고 유형, lane: 차선
int UARTIF_send_my_accident(uint8_t accident_type, uint8_t lane);

// [송신] 내 주행 방향(Heading) 전송 (WL-4 프로토콜)
int UARTIF_send_my_heading(uint16_t heading);

// [송신] Raw 바이트 전송 (추가됨)
// accident_send_module에서 패킷을 직접 만들어 보낼 때 사용
int UARTIF_send_raw_bytes(const uint8_t* data, uint32_t len);

#endif // UART_INTERFACE_H