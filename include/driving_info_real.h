#ifndef DRIVING_INFO_MODULE_H
#define DRIVING_INFO_MODULE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================
//  Data Types
// =============================================================

// 차선 정보 (1~3차선)
typedef enum {
    DIM_LANE_UNKNOWN = 0,
    DIM_LANE_1 = 1,
    DIM_LANE_2 = 2,
    DIM_LANE_3 = 3
} dim_lane_t;

// 전방 객체 종류 (AI 인식 결과)
typedef enum {
    DIM_OBJ_NONE = 0,
    DIM_OBJ_OBSTACLE,  // 장애물
    DIM_OBJ_CONE,      // 라바콘
    DIM_OBJ_UNKNOWN
} dim_obj_type_t;

// 전방 객체 상세 정보 구조체
typedef struct {
    dim_obj_type_t type;
    uint8_t  confidence_pct;  // 신뢰도 (0~100)
    int16_t  rel_lane;        // 상대 차선 (-1:좌, 0:내차선, +1:우)
    uint16_t dist_cm;         // 거리 (cm)
} dim_front_object_t;

// 차량 전체 상태 스냅샷 (모든 정보를 한 번에 읽기 위함)
typedef struct {
    dim_lane_t lane;              // 현재 주행 차선
    dim_front_object_t front_obj; // 전방 객체 정보
    
    uint32_t car_speed_1e5_mps;   // 차량 속도 (단위: 0.00001 m/s)
    uint16_t ultra_dist_mm;       // 초음파 센서 거리 (mm)
    uint16_t heading_deg;         // 주행 방향 (도)

    double gps_lat;               // 위도
    double gps_lon;               // 경도

    // 데이터 신선도 확인용 타임스탬프 (ms)
    uint32_t ts_lane_ms;
    uint32_t ts_obj_ms;
    uint32_t ts_speed_ms;
    uint32_t ts_ultra_ms;
    uint32_t ts_heading_ms;

    uint32_t seq;                 // 데이터 업데이트 카운터
} dim_snapshot_t;

// =============================================================
//  Public API Functions
// =============================================================

// 모듈 초기화 및 종료
int  DIM_init(void);
void DIM_deinit(void);

// [데이터 업데이트] - 센서나 로직에서 호출
void DIM_update_lane(dim_lane_t lane);
void DIM_update_speed(uint32_t speed_1e5_mps);
void DIM_update_ultrasonic(uint16_t dist_mm);
void DIM_update_heading(uint16_t heading_deg);
void DIM_update_gps(double lat, double lon);

// [객체 업데이트 1] 구조체 전체를 통째로 업데이트할 때 사용
void DIM_update_front_object(const dim_front_object_t* obj);

// [객체 업데이트 2] AI 수신 데이터(Raw)를 바로 업데이트할 때 사용 (편의 함수)
// 내부에서 dim_front_object_t를 생성해서 업데이트함
void DIM_update_front_obj(uint8_t type, uint16_t dist_cm);

// [데이터 조회] - 제어/판단 모듈에서 호출
int  DIM_get_snapshot(dim_snapshot_t* out); // 전체 데이터 복사 (Thread-Safe)

// 개별 데이터 조회 (Thread-Safe)
dim_lane_t DIM_get_lane(void);
uint32_t   DIM_get_speed(void);
uint16_t   DIM_get_ultrasonic(void);
uint16_t   DIM_get_heading(void);
int        DIM_get_front_object(dim_front_object_t* out);

// [데이터 신선도 체크] - 너무 오래된 데이터인지 확인 (ms 단위)
// return: 1(신선함), 0(오래됨)
int DIM_is_fresh_lane(uint32_t max_age_ms);
int DIM_is_fresh_speed(uint32_t max_age_ms);
int DIM_is_fresh_ultra(uint32_t max_age_ms);
int DIM_is_fresh_heading(uint32_t max_age_ms);
int DIM_is_fresh_obj(uint32_t max_age_ms);

#ifdef __cplusplus
}
#endif

#endif // DRIVING_INFO_MODULE_H