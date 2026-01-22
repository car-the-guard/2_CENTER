#define _GNU_SOURCE
#include "bluetooth_interface_real.h"
#include <pthread.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

static bt_config_t g_cfg;
static bt_on_cmd_fn g_cb = NULL;
static pthread_t g_thr;
static int g_running = 0;
static int g_fd = -1;

// Baudrate 변환 함수
static speed_t baud_to_speed(int baud) {
    switch(baud) {
        case 9600: return B9600;
        case 115200: return B115200;
        default: return B9600;
    }
}

static int internal_bt_open(const char *dev, int baud) {
    // [수정 1] O_RDONLY -> O_RDWR (읽기/쓰기 모드 권장)
    // D3-G에서 안정적인 포트 점유를 위해 수정
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd < 0) {
        perror("[BT] Open Error"); // 구체적인 에러 출력
        return -1;
    }

    // 통신 설정을 위해 잠시 Blocking을 풉니다.
    fcntl(fd, F_SETFL, 0);

    struct termios t;
    if(tcgetattr(fd, &t) < 0) { close(fd); return -1; }
    
    // Raw Mode 설정
    cfmakeraw(&t);
    
    // Baudrate 설정
    cfsetospeed(&t, baud_to_speed(baud));
    cfsetispeed(&t, baud_to_speed(baud));
    
    // [설정] 1바이트라도 들어올 때까지 대기 (Blocking Read)
    t.c_cc[VMIN] = 1; 
    t.c_cc[VTIME] = 0; 
    
    if(tcsetattr(fd, TCSANOW, &t) < 0) { close(fd); return -1; }
    
    // 플러시 (기존에 쌓인 데이터 삭제)
    tcflush(fd, TCIOFLUSH);

    return fd;
}

static void* bt_rx_thread(void* arg) {
    (void)arg;
    
    // UART 포트 열기
    g_fd = internal_bt_open(g_cfg.uart_dev, g_cfg.baud);
    if(g_fd < 0) { 
        fprintf(stderr, "[BT] Failed to open %s\n", g_cfg.uart_dev); 
        return NULL; 
    }
    printf("[[[[[[SMARTPHONE CONTROL ACTIVATED]]]]]] %s @ %d bps\n", g_cfg.uart_dev, g_cfg.baud);

    // 고스트 데이터 제거 대기
    usleep(500000);
    tcflush(g_fd, TCIOFLUSH);

    // 수신 버퍼
    char buf[128];
    int idx = 0;

    while(g_running) {
        char ch;
        // 1바이트 읽기 (Blocking)
        int n = read(g_fd, &ch, 1);
        
        if(n > 0) {
            // [데이터 수신 성공]
            // 개행 문자(\n, \r)를 만나면 명령 완성으로 판단
            if (ch == '\n' || ch == '\r') {
                if (idx > 0) {
                    buf[idx] = '\0'; // 문자열 종료
                    printf("[BT] Recv CMD: %s\n", buf); // 디버그용
                    if (g_cb) g_cb(buf); // 콜백 호출 (daemon2.c로 전달)
                    idx = 0; // 버퍼 초기화
                }
            } 
            else {
                // 버퍼에 담기 (Overflow 방지)
                if (idx < (int)(sizeof(buf) - 1)) {
                    buf[idx++] = ch;
                }
            }
        } 
        else if (n < 0) {
            // [수정 2] 에러 처리 강화
            if (errno == EINTR) continue; // 시스템 콜 중단이면 재시도
            // 진짜 에러인 경우
            // perror("[BT] Read Error");
            usleep(10000);
        }
    }

    if(g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
    return NULL;
}

// ---------------- Public API ----------------

int BT_init(const bt_config_t* cfg, bt_on_cmd_fn cb) {
    if(!cfg) return -1;
    g_cfg = *cfg; 
    g_cb = cb;
    return 0;
}

int BT_start(void) {
    if(g_running) return 0;
    g_running = 1;
    if(pthread_create(&g_thr, NULL, bt_rx_thread, NULL) != 0) {
        g_running = 0;
        return -1;
    }
    return 0;
}

void BT_stop(void) {
    if (!g_running) return;
    g_running = 0;
    
    // Blocking read를 깨우기 위해 강제 종료 시도
    if (g_fd >= 0) {
        // 원래는 cancel이나 signal을 써야 하지만, 
        // close()를 호출하면 read()가 에러를 뱉고 루프를 빠져나오게 유도함
        close(g_fd); 
        g_fd = -1;
    }
    pthread_join(g_thr, NULL);
    printf("[BT] Stopped.\n");
}