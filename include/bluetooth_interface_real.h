#ifndef BLUETOOTH_MODULE_H
#define BLUETOOTH_MODULE_H

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================
//  Data Structures
// =============================================================

// [설정] 블루투스 모듈 초기화 파라미터
typedef struct {
    const char* uart_dev;   // UART 장치 경로 (예: "/dev/ttyAMA1")
    int baud;               // 통신 속도 (예: 9600)
    int enable_stdout;      // 로그 출력 여부 (1:ON, 0:OFF)
} bt_config_t;

// [콜백] 명령어 수신 시 호출될 함수 타입
// cmd_str: 수신된 문자열 (개행 문자 제외된 순수 커맨드)
typedef void (*bt_on_cmd_fn)(const char* cmd_str);

// =============================================================
//  Public API Functions
// =============================================================

// 모듈 초기화
int  BT_init(const bt_config_t* cfg, bt_on_cmd_fn cb);

// 모듈 시작 (수신 스레드 생성)
int  BT_start(void);

// 모듈 정지 (스레드 종료 및 리소스 해제)
void BT_stop(void);

#ifdef __cplusplus
}
#endif

#endif // BLUETOOTH_MODULE_H