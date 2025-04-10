#ifndef DL_TELEMETRY_H
#define DL_TELEMETRY_H

#include <stdint.h>
#include "dl_general.h"

#define TELEMETRY_NUM_FLAGS     2
#define TELEMETRY_DATA_LENGTH   14
#define TELEMETRY_PORT          0x0F
#define TELEMETRY_CHANNEL       7

void telemetrySendMode(const sensor_data_t *sensor);
void write_u16(uint8_t *dst, uint8_t *idx, uint16_t value);
void write_i16(uint8_t *dst, uint8_t *idx, int16_t value);

void telemetryRecMode(void);

#endif // DL_TELEMETRY_H