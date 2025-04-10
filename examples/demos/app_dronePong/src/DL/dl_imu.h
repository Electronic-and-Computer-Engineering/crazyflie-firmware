#ifndef DL_IMU_H
#define DL_IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "dl_general.h"

void dl_imu_init(void);
bool dl_imu_read_yaw(sensor_data_t *data);
void updateGyroFiltered(void);

#endif // DL_IMU_H