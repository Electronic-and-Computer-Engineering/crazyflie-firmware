#include <string.h>

#include "dl_general.h"
#include "dl_telemetry.h"
#include "dl_flowdeck.h"
#include "dl_multiranger.h"
#include "dl_imu.h"
#include "dl_aiDeck.h"

#include "debug.h"

sensor_data_t   sensorDecks;
drone_data_t    droneData;
State state = IDLE;

void DL_init(void) 
{
    sensorDecks.armed = false;
    sensorDecks.arm = false;

    telemetryRecMode();
    dl_multiranger_init();
    dl_flowdeck_init();
    dl_imu_init();
    aiDeck_init();
}