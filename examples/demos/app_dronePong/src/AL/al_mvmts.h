#ifndef AL_MVMNTS_H
#define AL_MVMNTS_H

#include "stabilizer_types.h"
#include "dl_general.h"

void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate);
void setStabilizer(multiranger_data_t *rngData, drone_data_t *drnData);
void setStabilizerDir(multiranger_data_t *rngData, drone_data_t *drnData);

#endif // AL_MVMNTS_H
