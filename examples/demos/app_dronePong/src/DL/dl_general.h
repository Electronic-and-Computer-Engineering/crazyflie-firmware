#ifndef DL_GENERAL_H
#define DL_GENERAL_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t front;
    uint16_t back;
    uint16_t left;
    uint16_t right;
    uint16_t up;
} multiranger_data_t;

typedef struct {
    uint16_t z_mm;     // Höhe in mm
    int16_t vx;        // Bewegung in X (in px/s, signed!)
    int16_t vy;        // Bewegung in Y
} flow_data_t;

typedef struct {
    float   f_yaw;
    int16_t iscaledYaw;
} imu_data_t;

typedef struct {
    uint16_t droneFlags;
    multiranger_data_t multiranger;
    flow_data_t flow;
    imu_data_t imuData;
    bool armed;
    bool arm;
} sensor_data_t;

typedef enum {
    IDLE,
    HOVER,
    RAMPDOWN,
    ARM,
    UNARM,
    LAND,
    ROTATE180,
    ROTATE0,
    ROTATE180DIR,
    ROTATE0DIR,
    DIRECTIONAL
} State;

typedef struct {
    float l_comp;
    float r_comp;
    float velSide;
    float f_comp;
    float b_comp;
    float velFront;
    float height;
    float des_height;
    float degree;
    float dirAngle;
    float desSpeed;
} drone_data_t;

void DL_init(void);

#endif // DL_GENERAL_H