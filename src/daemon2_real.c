// src/daemon2.c
#define _GNU_SOURCE

// ===== AI 패킷 헤더 정의 =====
#define HEADER_AI_LANE    0xA1
#define HEADER_AI_OBJ     0xA2

// 117 -> D3-G 물리 37번 핀
#define BUZZER_GPIO 117

#include <errno.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <ctype.h>
#include <fcntl.h>
#include <termios.h>

static volatile int g_trigger_buzzer = 0;
static pthread_mutex_t g_buz_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_buz_cv  = PTHREAD_COND_INITIALIZER;

// ===== 모듈 헤더 포함 =====
#include "driving_info_real.h"
#include "bluetooth_interface_real.h"
#include "can_interface_real.h"
#include "collision_risk_real.h"
#include "accident_send_real.h"
#include "collision_response_real.h"
#include "uart_interface_real.h" // TODO: 센터와 모든 통신을 CAN으로 할 것 같다?

// ===== 설정 구조체 =====
typedef struct {
    int enable_stdout;
    const char* can_ifname;
    const char* bt_uart_dev;   // Bluetooth
    const char* wl_uart_dev;   // WL 통신
    const char* ai_uart_dev;   // AI 수신
    int common_baud;
} daemon_cfg_t;

// ===== 전역 변수 =====
static volatile sig_atomic_t g_stop = 0;
static float g_real_speed_mps = 0.0f;
static int g_is_aeb_active = 0;

// AI 수신 파싱 변수
static int rx_state = 0;
static uint8_t rx_packet_type = 0;
static uint8_t rx_buffer[10];
static int rx_buf_idx = 0;
static int rx_target_len = 0;

static pthread_t g_ai_thr;
static pthread_t g_buzzer_thr;

// ===== 유틸리티 함수 =====
static uint32_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

static uint32_t bytes_to_uint32(uint8_t* b) {
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
           ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

static void on_sig(int sig) {
    (void)sig;
    g_stop = 1;
}

// [TODO] D3-G용 GPIO 제어 함수로 교체 필요 (sysfs 방식은 동일할 수 있음)
static void gpio_export(int gpio) {
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd != -1) {
        dprintf(fd, "%d", gpio);
        close(fd);
    }
}

static void gpio_set_dir(int gpio, int out) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
    int fd = open(path, O_WRONLY);
    if (fd != -1) {
        write(fd, out ? "out" : "in", out ? 3 : 2);
        close(fd);
    }
}

static void gpio_set_value(int gpio, int value) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    int fd = open(path, O_WRONLY);
    if (fd != -1) {
        write(fd, value ? "1" : "0", 1);
        close(fd);
    }
}

static void* buzzer_task(void* arg) {
    (void)arg;
    gpio_export(BUZZER_GPIO);
    usleep(100000);
    gpio_set_dir(BUZZER_GPIO, 1);
    gpio_set_value(BUZZER_GPIO, 0);

    while (!g_stop) {
        pthread_mutex_lock(&g_buz_mtx);
        while (!g_trigger_buzzer && !g_stop) {
            pthread_cond_wait(&g_buz_cv, &g_buz_mtx);
        }
        g_trigger_buzzer = 0;
        pthread_mutex_unlock(&g_buz_mtx);

        if (g_stop) break;

        printf("********** BUZZER ON **********\n");
        gpio_set_value(BUZZER_GPIO, 1);
        usleep(1000000);
        gpio_set_value(BUZZER_GPIO, 0);
    }
    gpio_set_value(BUZZER_GPIO, 0);
    return NULL;
}

// ===== AI 데이터 파싱 로직 =====
static void parse_ai_byte(uint8_t byte) {
    if (rx_state == 0) {
        if (byte == HEADER_AI_LANE) {
            rx_packet_type = HEADER_AI_LANE;
            rx_state = 1;
            rx_buf_idx = 0;
            rx_target_len = 1 + 4;
        }
        else if (byte == HEADER_AI_OBJ) {
            rx_packet_type = HEADER_AI_OBJ;
            rx_state = 1;
            rx_buf_idx = 0;
            rx_target_len = 2 + 4;
        }
    }
    else if (rx_state == 1) {
        rx_buffer[rx_buf_idx++] = byte;

        if (rx_buf_idx >= rx_target_len) {
            int time_idx = rx_target_len - 4;
            // uint32_t timestamp = bytes_to_uint32(&rx_buffer[time_idx]); // Unused variable removed
            uint32_t now = now_ms();

            if (rx_packet_type == HEADER_AI_LANE) {
                uint8_t lane = rx_buffer[0];
                if (lane > 0 && lane <= 3) {
                    DIM_update_lane(lane);
                    static uint32_t last_print_lane = 0;
                    if (now - last_print_lane >= 1000) {
                        last_print_lane = now;
                        printf("*******************LANE UPDATED: %d *******************\n", lane);
                    }
                }
            }
            else if (rx_packet_type == HEADER_AI_OBJ) {
                uint8_t type = rx_buffer[0];

                // AI 객체 타입 업데이트
                static uint32_t last_print_obj = 0;
                if (now - last_print_obj >= 1000) {
                    last_print_obj = now;
                    printf("*******************AI OBJ TYPE: %d *******************\n", type);
                }
            }
            rx_state = 0;
            rx_buf_idx = 0;
        }
    }
}

// ===== 콜백 함수들 =====
static void on_uart_external_data(uint8_t* data, uint16_t len) {
    (void)data; (void)len;
    // WL 프로토콜 수신 처리 (필요 시 구현)
}

// CAN 수신 콜백 함수 초음파
static void on_can_ultra_cm(uint16_t raw_mm) {
    DIM_update_ultrasonic(raw_mm);
}

static void on_can_speed(uint32_t v_cmps) {
    int16_t speed_cmps = (int16_t)v_cmps;
    g_real_speed_mps = (float)speed_cmps / 100.0f;
    uint32_t speed_dim_unit = (uint32_t)(g_real_speed_mps * 100000.0f);
    DIM_update_speed(speed_dim_unit);
}

static void on_can_accel_x(int16_t raw, int16_t filt) { (void)raw; (void)filt; }
static void on_can_heading(uint16_t deg) { DIM_update_heading(deg); }

static void on_can_vibration(void) {
    static uint32_t last_crash_report_ms = 0;
    uint32_t now = now_ms();

    // printf("*******************COLLISION DETECTED*******************\n");

    pthread_mutex_lock(&g_buz_mtx);
    g_trigger_buzzer = 1;
    pthread_cond_signal(&g_buz_cv);
    pthread_mutex_unlock(&g_buz_mtx);

    if (now - last_crash_report_ms >= 1000) {
        last_crash_report_ms = now;

        // 현재 차선 가져오기
        uint8_t current_lane = DIM_get_lane();
        if (current_lane == 0) current_lane = 1;

        UARTIF_send_my_accident(1, current_lane);

        printf("[WL-3] CRASH REPORT SENT! (Lane: %d)\n", current_lane);
    }
}

// 충돌위험판단
static void cb_crm_brake_lamp(int level) {
    canif_brake_mode_t mode = BRAKE_OFF;
    if (level == 1) mode = BRAKE_ON;
    else if (level == 2) mode = BRAKE_BLINK;
    else if (level == 3) mode = (canif_brake_mode_t)3; // Arrow Animation

    CANIF_send_brakelight(mode, (uint16_t)(now_ms() & 0xFFFF));
}

// AEB 제어 콜백
static void cb_crm_aeb(int enable) {
    uint16_t t = (uint16_t)(now_ms() & 0xFFFF);

    if (enable) {
        g_is_aeb_active = 1;
        printf("******************* AEB TRIGGERED *******************\n");
        CANIF_send_aeb_enable(t);

        printf("******************* SEATBELT PRETENSION *******************\n");
        CANIF_send_seatbelt(1);
    }
    else {
        g_is_aeb_active = 0;
        CANIF_send_aeb_disable(t);
        CANIF_send_seatbelt(0);
    }
}

static void cb_crm_airbag(int enable) { (void)enable; }

static void cb_crm_notify_scale(int scale) {
    acc_event_t ev = { .severity = scale, .type = ACC_TYPE_HARD_BRAKE };
    ACCSEND_on_event(&ev);
}

static void cb_crm_seatbelt(int enable) {
    // 필요 시 CANIF_send_seatbelt(enable); 호출
    (void)enable;
}

static void on_bt_cmd(const char* cmd) {
    static int current_speed = 0;
    static int current_turn = 0;

    const char* p = cmd;
    while (*p) {
        char c = *p;
        if (isdigit(c) || c == '-' || c == ' ' || c == '\n' || c == '\r') {
            p++; continue;
        }
        int val = atoi(p + 1);

        if      (c == 'f' || c == 'F') current_speed = val;
        else if (c == 'b' || c == 'B') current_speed = -val;
        else if (c == 'r' || c == 'R') current_turn = val;
        else if (c == 'l' || c == 'L') current_turn = -val;
        p++;
    }

    if (g_is_aeb_active) {
        current_speed = 0;
        printf("******************* AEB ON Ignoring BT CONTROL *******************\n");
    }

    if (current_speed > 100) current_speed = 100;
    if (current_speed < -100) current_speed = -100;
    if (current_turn > 100) current_turn = 100;
    if (current_turn < -100) current_turn = -100;

    canif_motor4_t m = { .ch = {0,0,0,0} };
    m.ch[0] = (int8_t)current_speed;

    if (current_turn < 0) {
        m.ch[2] = (int8_t)(-current_turn);
        m.ch[3] = 0;
    } else {
        m.ch[2] = 0;
        m.ch[3] = (int8_t)(current_turn);
    }
    CANIF_send_motor4(&m, (uint16_t)(now_ms() & 0xFFFF));
}

// ===== 스레드: AI UART 수신 =====
static void* ai_rx_thread(void* arg) {
    const char* dev = (const char*)arg;
    printf("[AI-THREAD] Opening %s at 9600 bps...\n", dev);

    int fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("[AI-THREAD] OPEN FAILED BECAUSE IM NOT CONNECTED TO AI MODULE YET!");
        return NULL;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        perror("[AI-THREAD] tcgetattr");
        close(fd);
        return NULL;
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("[AI-THREAD] tcsetattr");
        close(fd);
        return NULL;
    }

    printf("[AI-THREAD] Started Listening on %s\n", dev);

    uint8_t buf[128];
    while (!g_stop) {
        int len = read(fd, buf, sizeof(buf));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                parse_ai_byte(buf[i]);
            }
        }
        usleep(1000);
    }
    close(fd);
    return NULL;
}

// =============================================================
// MAIN FUNCTION
// =============================================================
int main(int argc, char** argv)
{
    daemon_cfg_t cfg = {
        .enable_stdout = 1,
        
        // TODO: D3-G TTYAMA 어디에 연결할지 확인 필요
        .can_ifname = "can0",
        .bt_uart_dev = "/dev/ttyAMA1",
        .wl_uart_dev = "/dev/ttyAMA3", // 나중에 수정
        .ai_uart_dev = "/dev/ttyAMA5", // 나중에 수정
        .common_baud = 9600
    };

    static struct option long_opts[] = {
        {"can-if", required_argument, 0, 1},
        {"bt-dev", required_argument, 0, 2},
        {0,0,0,0}
    };
    int idx, c;
    while((c = getopt_long(argc, argv, "", long_opts, &idx)) != -1) {
        switch(c) {
            case 1: cfg.can_ifname = optarg; break;
            case 2: cfg.bt_uart_dev = optarg; break;
        }
    }

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    printf("**************************************** APP_MAIN STARTED ****************************************\n");

    pthread_create(&g_buzzer_thr, NULL, buzzer_task, NULL);
    printf(" - CAN: %s\n", cfg.can_ifname);
    printf(" - BT : %s\n", cfg.bt_uart_dev);
    printf(" - WL : %s\n", cfg.wl_uart_dev);
    printf(" - AI : %s\n", cfg.ai_uart_dev);

    printf("****************************************                  ****************************************\n");

    DIM_init();

    canif_rx_handlers_t can_rxh = {
        .on_ultrasonic_cm = on_can_ultra_cm,
        .on_speed_q4 = on_can_speed,
        .on_accel_x = on_can_accel_x,
        .on_heading_deg = on_can_heading,
        .on_vibration = on_can_vibration
    };
    canif_config_t can_cfg = { .ifname = cfg.can_ifname, .motor_map = {0,1,2,3} };
    if(CANIF_init(&can_cfg, &can_rxh) != 0) fprintf(stderr, "CANIF Init Failed\n");

    crm_callbacks_t crm_cb = {
        .send_brake_lamp_level = cb_crm_brake_lamp,
        .send_aeb = cb_crm_aeb,
        .notify_accident_scale = cb_crm_notify_scale,
        .deploy_airbag = cb_crm_airbag,
        .control_seatbelt = cb_crm_seatbelt
    };
    crm_config_t crm_cfg = { .period_ms=50, .max_data_age_ms=500 };
    CRM_init(&crm_cfg, &crm_cb);

    bt_config_t bt_cfg = { .uart_dev = cfg.bt_uart_dev, .baud = cfg.common_baud, .enable_stdout = 1 };
    if(BT_init(&bt_cfg, on_bt_cmd) != 0) fprintf(stderr, "BT Init Failed\n");

    uartif_config_t uart_cfg = { .dev_path = cfg.wl_uart_dev, .baudrate = cfg.common_baud };
    uartif_rx_handlers_t uart_hdl = { .on_data = on_uart_external_data };

    if (UARTIF_init(&uart_cfg, &uart_hdl) == 0) {
        UARTIF_start();
        printf("[MAIN] WL UART Started on %s\n", uart_cfg.dev_path);
    }

    CANIF_start();
    CRM_start();
    BT_start();

    pthread_create(&g_ai_thr, NULL, ai_rx_thread, (void*)cfg.ai_uart_dev);

    uint32_t last_step = 0;
    uint32_t start_time = now_ms();
    uint32_t last_wl4_tx = 0;

    // TODO: 지자기는 충돌했을 때만 전송하도록 변경 필요 (저거 주석 처리?)
    // ================= LOOP =================
    while(!g_stop) {
        uint32_t now = now_ms();
        uint32_t elapsed = now - start_time;

        // [시나리오 1] 지자기(WL-4) 전송 (1초 간격)
        // if (elapsed > 10000 && (now - last_wl4_tx >= 1000)) {
        //     last_wl4_tx = now;
        //     static uint16_t sim_heading = 30;
        //     UARTIF_send_my_heading(sim_heading);
        //     printf("[WL-4] 지자기 UART: %d deg\n", sim_heading);
        // }

        // [기본] 센서 데이터 업데이트 및 충돌 판단 (50ms)
        if((now - last_step) >= 50) {
            last_step = now;
            dim_snapshot_t snap;
            if(DIM_get_snapshot(&snap) == 0) {
                crm_inputs_t in;
                in.dist_m = snap.ultra_dist_mm / 1000.0f;
                in.rel_speed_mps = snap.car_speed_1e5_mps / 100000.0f;
                in.timestamp_ms = now;
                CRM_update_inputs(&in);
            }
        }
        usleep(5000);
    }

    printf("STOPPING...\n");

    pthread_mutex_lock(&g_buz_mtx);
    pthread_cond_signal(&g_buz_cv);
    pthread_mutex_unlock(&g_buz_mtx);

    pthread_join(g_buzzer_thr, NULL);
    BT_stop();
    CRM_stop();
    CANIF_stop();
    DIM_deinit();
    UARTIF_stop();

    return 0;
}