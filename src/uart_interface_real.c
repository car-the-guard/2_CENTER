#include "uart_interface_real.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <errno.h>

#define WL3_UART_HEADER  (3) // 기존 코드 유지
#define WL4_UART_HEADER  (4) // Heading용 헤더 (추정)

// 내부 변수
static int g_uart_fd = -1;
static volatile int g_stop_flag = 0;
static pthread_t g_rx_thread;
static uartif_rx_handlers_t g_handlers = {0};

// 수신 스레드 함수
static void* rx_thread_func(void* arg) {
    (void)arg;
    uint8_t rx_buffer[256];

    while (!g_stop_flag) {
        if (g_uart_fd < 0) {
            usleep(100000);
            continue;
        }

        int len = read(g_uart_fd, rx_buffer, sizeof(rx_buffer));
        if (len > 0) {
            if (g_handlers.on_data) {
                g_handlers.on_data(rx_buffer, (uint16_t)len);
            }
        }
        usleep(1000); 
    }
    return NULL;
}

int UARTIF_init(const uartif_config_t* cfg, const uartif_rx_handlers_t* handlers) {
    if (!cfg || !cfg->dev_path) return -1;

    if (handlers) {
        g_handlers = *handlers;
    }

    // D3-G UART 포트 열기
    // O_NDELAY: Non-blocking 모드로 열림 (read시 바로 리턴)
    g_uart_fd = open(cfg->dev_path, O_RDWR | O_NOCTTY | O_NDELAY);
    if (g_uart_fd < 0) {
        // cfg->dev_path가 NULL이거나 잘못된 경로일 경우 여기서 잡힘
        perror("Message from UART_INTERFACE_REAL [NEED TO CONNECT WITH WL D3-G]"); 
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(g_uart_fd, &tty) != 0) {
        perror("[UARTIF] tcgetattr");
        close(g_uart_fd);
        g_uart_fd = -1;
        return -1;
    }

    speed_t speed;
    switch(cfg->baudrate) {
        case 9600: speed = B9600; break;
        case 115200: speed = B115200; break;
        default: speed = B9600; 
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1 설정
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS; // Flow Control 끔
    
    // Raw Mode (아주 중요)
    tty.c_lflag = 0; 
    tty.c_oflag = 0; 
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); 

    // 읽기 타임아웃 설정 (Non-blocking이지만 최소한의 설정)
    tty.c_cc[VMIN] = 0;  
    tty.c_cc[VTIME] = 1; 

    tty.c_cflag |= (CLOCAL | CREAD);

    if (tcsetattr(g_uart_fd, TCSANOW, &tty) != 0) {
        perror("[UARTIF] tcsetattr");
        close(g_uart_fd);
        g_uart_fd = -1;
        return -1;
    }

    // [중요] Blocking 모드로 변경 (read 호출 시 데이터 올 때까지 대기하거나 타임아웃)
    // CPU 점유율을 낮추기 위해 필요
    fcntl(g_uart_fd, F_SETFL, 0);

    return 0;
}

int UARTIF_start(void) {
    if (g_uart_fd < 0) return -1; // init 실패했으면 시작 안 함

    g_stop_flag = 0;
    if (pthread_create(&g_rx_thread, NULL, rx_thread_func, NULL) != 0) {
        perror("[UARTIF] Thread Create Failed");
        return -1;
    }
    return 0;
}

void UARTIF_stop(void) {
    g_stop_flag = 1;
    if (g_rx_thread) {
        pthread_join(g_rx_thread, NULL);
    }
    if (g_uart_fd >= 0) {
        close(g_uart_fd);
        g_uart_fd = -1;
    }
}

// [WL-3] 내 사고 정보 전송
// 불필요한 I2C 코드 주석 제거 및 구조체 사용으로 깔끔하게 정리
typedef struct __attribute__((packed)) {
    uint8_t  accident_type;
    uint16_t debug_time;
    uint8_t  sev_action;
    uint8_t  lane;
    uint16_t dir_rsv;
    uint64_t accident_time;
    uint64_t accident_id;
} wl3_packet_t;

int UARTIF_send_my_accident(uint8_t accident_type, uint8_t lane)
{
    if (g_uart_fd < 0) return -1;

    wl3_packet_t p;
    memset(&p, 0, sizeof(p));

    p.accident_type  = accident_type;
    p.lane           = lane;
    p.accident_id    = 0x9999999999999999ULL; // 임시 ID

    uint8_t buf[1 + sizeof(wl3_packet_t)];
    buf[0] = WL3_UART_HEADER; // 헤더 (3)
    memcpy(&buf[1], &p, sizeof(p));

    int sent = write(g_uart_fd, buf, sizeof(buf));
    return (sent == (int)sizeof(buf)) ? 0 : -1;
}

// [WL-4] 내 주행 정보(Heading) 전송
// [수정] 기존 코드의 배열 인덱스 오류 및 비트 연산 수정
// TODO: 여기 나중에 수정
int UARTIF_send_my_heading(uint16_t heading) {
    if (g_uart_fd < 0) return -1;

    // 패킷 구조: [HEADER] [Heading High] [Heading Low] (총 3바이트)
    // 기존 코드의 의도: 첫 바이트에 헤더(4)를 비트 OR 연산하거나 별도 헤더 사용
    // 여기서는 안전하게 3바이트 패킷으로 정의합니다.
    
    uint8_t packet[3];
    packet[0] = WL4_UART_HEADER;      // 헤더: 4
    packet[1] = (heading >> 8) & 0xFF; // 상위 바이트
    packet[2] = heading & 0xFF;        // 하위 바이트

    int sent = write(g_uart_fd, packet, 3);
    return (sent == 3) ? 0 : -1;
}

int UARTIF_send_raw_bytes(const uint8_t* data, uint32_t len) {
    if (g_uart_fd < 0) return -1;

    int sent = write(g_uart_fd, data, len);
    return (sent == (int)len) ? 0 : -1;
}