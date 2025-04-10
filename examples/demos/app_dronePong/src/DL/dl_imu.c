// dl_motion.c - Detect motion using Multiranger, Flowdeck and Gyro

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "dl_general.h"
#include "log.h"
#include "dl_imu.h"
#include "stabilizer.h"
#include "param.h"

#define DEG_TO_RAD (3.14159265f / 180.0f)
#define RAD_TO_DEG (180.0f / 3.14159265f)

#define FLOW_ENTER_STATIC_THRESHOLD  3.5f  // px/frame
#define FLOW_LEAVE_STATIC_THRESHOLD  6.0f

#define GYRO_ENTER_STATIC_THRESHOLD  2.5f  // deg/s
#define GYRO_LEAVE_STATIC_THRESHOLD  4.0f

#define MULTI_ENTER_STATIC_THRESHOLD 7.0f  // mm
#define MULTI_LEAVE_STATIC_THRESHOLD 12.0f

#define MEDIAN_DEPTH 3
#define YAW_THRESHOLD_DEG 2.0f

static float lastYawDeg = 0.0f;
static float frozenYawDeg = 0.0f;
static bool isStaticState = false;

// Previous multiranger values
static uint16_t prevFront = 0;
static uint16_t prevBack  = 0;
static uint16_t prevLeft  = 0;
static uint16_t prevRight = 0;

static Axis3f gyroFiltered = {0};
static logVarId_t id_gyro_x, id_gyro_y, id_gyro_z;
static paramVarId_t id_kalman_active;

// Median filter buffer for yaw
static float yawMedianBuffer[MEDIAN_DEPTH] = {0.0f};
static int yawMedianIndex = 0;
static bool yawMedianFull = false;

extern sensor_data_t   sensorDecks;

static float median(float *arr, int size)
{
    float tmp[MEDIAN_DEPTH];
    memcpy(tmp, arr, sizeof(float) * size);
    for (int i = 0; i < size-1; ++i) {
        for (int j = i+1; j < size; ++j) {
            if (tmp[i] > tmp[j]) {
                float t = tmp[i];
                tmp[i] = tmp[j];
                tmp[j] = t;
            }
        }
    }
    return tmp[size/2];
}

void updateGyroFiltered(void)
{
    float gx = logGetFloat(id_gyro_x);
    float gy = logGetFloat(id_gyro_y);
    float gz = logGetFloat(id_gyro_z);

    const float alpha = 0.9f;
    gyroFiltered.x = alpha * gyroFiltered.x + (1.0f - alpha) * gx;
    gyroFiltered.y = alpha * gyroFiltered.y + (1.0f - alpha) * gy;
    gyroFiltered.z = alpha * gyroFiltered.z + (1.0f - alpha) * gz;
}

static float computeFlowMagnitude(sensor_data_t *data)
{
    int16_t dx = data->flow.vx;
    int16_t dy = data->flow.vy;
    return sqrtf((float)(dx * dx + dy * dy));
}

static float computeGyroMagnitude(void)
{
    float gx = gyroFiltered.x;
    float gy = gyroFiltered.y;
    float gz = gyroFiltered.z;
    return sqrtf(gx * gx + gy * gy + gz * gz);
}

static float computeMultirangerMotion(sensor_data_t *data)
{
    int16_t dFront = (int16_t)prevFront - (int16_t)data->multiranger.front;
    int16_t dBack  = (int16_t)prevBack  - (int16_t)data->multiranger.back;
    int16_t dLeft  = (int16_t)prevLeft  - (int16_t)data->multiranger.left;
    int16_t dRight = (int16_t)prevRight - (int16_t)data->multiranger.right;

    prevFront = data->multiranger.front;
    prevBack  = data->multiranger.back;
    prevLeft  = data->multiranger.left;
    prevRight = data->multiranger.right;

    float motionX = (float)(dBack - dFront);
    float motionY = (float)(dRight - dLeft);
    return sqrtf(motionX * motionX + motionY * motionY);
}

bool dl_imu_read_yaw(sensor_data_t *data)
{
    updateGyroFiltered();

    float flowMag = computeFlowMagnitude(data);
    float gyroMag = computeGyroMagnitude();
    float multiMag = computeMultirangerMotion(data);

    if (isStaticState) {
        if (flowMag > FLOW_LEAVE_STATIC_THRESHOLD ||
            gyroMag > GYRO_LEAVE_STATIC_THRESHOLD ||
            multiMag > MULTI_LEAVE_STATIC_THRESHOLD) {
            isStaticState = false;
            sensorDecks.droneFlags = 0x10;
        }
    } else {
        if (flowMag < FLOW_ENTER_STATIC_THRESHOLD &&
            gyroMag < GYRO_ENTER_STATIC_THRESHOLD &&
            multiMag < MULTI_ENTER_STATIC_THRESHOLD) {
            isStaticState = true;
            frozenYawDeg = lastYawDeg;
            sensorDecks.droneFlags = 0x1E;
        }
    }

    float rawYaw = stabilizerGetYaw();
    float deltaRaw = rawYaw - lastYawDeg;
    if (deltaRaw > 180.0f) deltaRaw -= 360.0f;
    if (deltaRaw < -180.0f) deltaRaw += 360.0f;

    if (fabsf(deltaRaw) > YAW_THRESHOLD_DEG) {
        yawMedianBuffer[yawMedianIndex++] = rawYaw;
        if (yawMedianIndex >= MEDIAN_DEPTH) {
            yawMedianIndex = 0;
            yawMedianFull = true;
        }
    }

    int N = yawMedianFull ? MEDIAN_DEPTH : yawMedianIndex;
    float currentYaw = median(yawMedianBuffer, N);

    float delta = currentYaw - lastYawDeg;
    // if (delta > 180.0f) delta -= 360.0f;
    // if (delta < -180.0f) delta += 360.0f;

    if (fabsf(delta) > YAW_THRESHOLD_DEG) {
        lastYawDeg = currentYaw;
    }

    data->imuData.f_yaw = isStaticState ? frozenYawDeg : lastYawDeg;
    data->imuData.iscaledYaw = (int16_t)((data->imuData.f_yaw / 180.0f) * 32767.0f);

    return true;
}

void dl_imu_init(void)
{
    // Ensure Kalman estimator is used
    id_kalman_active = paramGetVarId("stabilizer", "estimator");
    paramSetInt(id_kalman_active, 2);

    prevFront = prevBack = prevLeft = prevRight = 0;

    id_gyro_x = logGetVarId("gyro", "xRaw");
    id_gyro_y = logGetVarId("gyro", "yRaw");
    id_gyro_z = logGetVarId("gyro", "zRaw");

    gyroFiltered = (Axis3f){0};
    yawMedianIndex = 0;
    yawMedianFull = false;
    lastYawDeg = stabilizerGetYaw();
    frozenYawDeg = lastYawDeg;
    isStaticState = false;
    memset(yawMedianBuffer, 0, sizeof(yawMedianBuffer));
}