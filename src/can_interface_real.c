// can_interface.c
#define _GNU_SOURCE
#include "can_interface_real.h"

#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

static int g_can = -1;
static pthread_t g_rx_thr;
static int g_running = 0;

static canif_config_t g_cfg;
static canif_rx_handlers_t g_rxh;

// CRC-8: 지금은 “자리만 확보”해 둔 상태. (byte7)
static uint8_t crc8_stub(const uint8_t* data, size_t len) {
    (void)data; (void)len;
    return 0x00;
}

static int open_can(const char* ifname) {
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) return -1;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        close(s);
        return -1;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(s);
        return -1;
    }
    return s;
}

static void payload_from_frame(const struct can_frame* fr, canif_payload_t* out) {
    // fr.data[0..7] -> payload
    memcpy(out, fr->data, 8);
}

static void payload_to_frame(uint16_t canid, const canif_payload_t* p, struct can_frame* fr) {
    memset(fr, 0, sizeof(*fr));
    fr->can_id  = (canid & 0x7FF);
    fr->can_dlc = 8;
    memcpy(fr->data, p, 8);
}

static void handle_rx(uint16_t canid, const canif_payload_t* p) {
    // data/time/reserved/crc 포맷 공통 
    switch (canid) {
    case CANID_VIBRATION:
        // data[0]=0xFF 충돌 신호
        if ((p->data & 0xFFu) == 0xFFu) {
            if (g_rxh.on_vibration) g_rxh.on_vibration();
        }
        break;

    case CANID_ULTRASONIC: {

        // uint16_t dist_cm = (uint16_t)(p->data & 0xFFFFu);
        // uint16_t dist_mm = dist_cm * 10;

        uint16_t raw = (uint16_t)(p->data & 0xFFFFu);
        uint16_t dist_cm = (raw >> 8) | (raw << 8);
        uint16_t dist_mm = dist_cm * 10;
        // printf("[CANIF] Recv UltraSonic: =%u cm\n",dist_cm);
        if (g_rxh.on_ultrasonic_cm) g_rxh.on_ultrasonic_cm(dist_mm);
        break;
    }

    case 0x13: {
            // Data[0], Data[1]을 합쳐서 int16_t (cm/s)로 변환
            int16_t speed_cmps = (int16_t)(p->data & 0xFFFFu);
            
            // 기존에 있던 속도 콜백 함수(on_speed_q4)를 재활용해서 메인으로 보냄
            // (형변환을 통해 int16_t 값을 uint32_t 파라미터에 실어 보냅니다)
            if (g_rxh.on_speed_q4) {
                g_rxh.on_speed_q4((uint32_t)speed_cmps); 
            }
            break;
        }

    case CANID_SPEED: {
        // 4바이트 속도정보. 단위 0.0001 m/s
        if (g_rxh.on_speed_q4) g_rxh.on_speed_q4(p->data);
        break;
    }

    case CANID_ACCEL: {
        // raw 2B + filtered 2B (X축만) 
        int16_t raw  = (uint16_t)(p->data & 0xFFFFu);
        int16_t filt = (uint16_t)((p->data >> 16) & 0xFFFFu);
        if (g_rxh.on_accel_x) g_rxh.on_accel_x(raw, filt);
        break;
    }

    case CANID_MAGNETIC: {
        // 0~359도. 문서상 2바이트 사용
        uint16_t deg = (uint16_t)(p->data & 0xFFFFu);
        if (g_rxh.on_heading_deg) g_rxh.on_heading_deg(deg);
        break;
    }

    default:
        break;
    }
}

static void* rx_thread(void* arg) {
    (void)arg;

    while (g_running) {

        struct can_frame fr;
        ssize_t n = read(g_can, &fr, sizeof(fr));
        if (n < 0) {
            if (errno == EINTR) continue;
            // CAN 내려가면 read가 깨질 수 있음. 여기서 바로 종료.
            break;
        }
        if (n != (ssize_t)sizeof(fr)) continue;

        // printf("[DEBUG] CAN Recv ID: 0x%X, DLC: %d\n", fr.can_id, fr.can_dlc);
        uint16_t canid = (uint16_t)(fr.can_id & 0x7FFu);
        if (fr.can_dlc != 8) continue;

        canif_payload_t p;
        payload_from_frame(&fr, &p);

        // CRC 검증은 아직 스텁 (byte7)
        // uint8_t exp = crc8_stub((uint8_t*)&p, 7);
        // if (p.crc8 != exp) continue;

        handle_rx(canid, &p);
    }
    return NULL;
}

int CANIF_init(const canif_config_t* cfg, const canif_rx_handlers_t* rxh) {
    if (!cfg || !cfg->ifname) return -1;

    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg = *cfg;

    memset(&g_rxh, 0, sizeof(g_rxh));
    if (rxh) g_rxh = *rxh;

    if (g_cfg.motor_map[0] == 0 && g_cfg.motor_map[1] == 0 &&
        g_cfg.motor_map[2] == 0 && g_cfg.motor_map[3] == 0) {
        // default mapping: 그대로
        g_cfg.motor_map[0] = 0;
        g_cfg.motor_map[1] = 1;
        g_cfg.motor_map[2] = 2;
        g_cfg.motor_map[3] = 3;
    }

    g_can = open_can(g_cfg.ifname);
    if (g_can < 0) return -1;

    return 0;
}

int CANIF_start(void) {
    if (g_can < 0) return -1;
    if (g_running) return 0;

    g_running = 1;
    if (pthread_create(&g_rx_thr, NULL, rx_thread, NULL) != 0) {
        g_running = 0;
        return -1;
    }
    return 0;
}

void CANIF_stop(void) {
    if (!g_running) return;
    g_running = 0;

    // read()를 깨우기 위해 소켓 닫고 join
    if (g_can >= 0) {
        close(g_can);
        g_can = -1;
    }
    pthread_join(g_rx_thr, NULL);
}

static int tx_send(uint16_t canid, canif_payload_t* p) {
    if (g_can < 0) return -1;

    p->crc8 = crc8_stub((uint8_t*)p, 7);

    struct can_frame fr;
    payload_to_frame(canid, p, &fr);

    ssize_t n = write(g_can, &fr, sizeof(fr));
    return (n == (ssize_t)sizeof(fr)) ? 0 : -1;
}

int CANIF_send_raw(uint16_t canid, const canif_payload_t* p) {
    if (!p) return -1;
    canif_payload_t tmp = *p;
    return tx_send(canid, &tmp);
}

int CANIF_send_aeb_enable(uint16_t time_ms) {
    // data0=0xFF (급정거)
    canif_payload_t p = {0};
    p.data    = 0x000000FFu;
    p.time_ms = time_ms;
    p.reserved = 0;
    return tx_send(CANID_AEB_CTRL, &p);
}

int CANIF_send_aeb_disable(uint16_t time_ms) {
    // data0=0x0F (급정거 해제)
    canif_payload_t p = {0};
    p.data    = 0x0000000Fu;
    p.time_ms = time_ms;
    p.reserved = 0;
    return tx_send(CANID_AEB_CTRL, &p);
}

int CANIF_send_brakelight(canif_brake_mode_t mode, uint16_t time_ms) {
    // data0 = 0/1/2/4 (OFF/ON/BLINK/YIELD)
    canif_payload_t p = {0};
    p.data    = ((uint32_t)mode & 0xFFu);
    p.time_ms = time_ms;
    p.reserved = 0;
    return tx_send(CANID_BRAKELIGHT, &p);
}

int CANIF_send_motor4(const canif_motor4_t* m, uint16_t time_ms) {
    if (!m) return -1;

    // payload.data byte0..3 구성:
    // byte[i] = m->ch[ motor_map[i] ]
    uint8_t b[4];
    b[0] = (uint8_t)m->ch[g_cfg.motor_map[0]];
    b[1] = (uint8_t)m->ch[g_cfg.motor_map[1]];
    b[2] = (uint8_t)m->ch[g_cfg.motor_map[2]];
    b[3] = (uint8_t)m->ch[g_cfg.motor_map[3]];

    canif_payload_t p = {0};
    p.data = (uint32_t)b[0]
           | ((uint32_t)b[1] << 8)
           | ((uint32_t)b[2] << 16)
           | ((uint32_t)b[3] << 24);
    p.time_ms = time_ms;
    p.reserved = 0;

    return tx_send(CANID_MOTOR_CTRL, &p);
}

// ID: 0x49, Data[0]: 1(ON)/0(OFF)
int CANIF_send_seatbelt(int enable) {
    canif_payload_t p = {0};
    
    // 0번 바이트에 1 또는 0 설정
    p.data = (enable ? 1 : 0); 
    
    // 필요하다면 타임스탬프 등 추가 (현재는 0)
    p.time_ms = 0; 
    p.reserved = 0;

    // tx_send 함수를 이용해 ID 0x49로 전송
    return tx_send(CANID_SEATBELT_CTRL, &p);
}