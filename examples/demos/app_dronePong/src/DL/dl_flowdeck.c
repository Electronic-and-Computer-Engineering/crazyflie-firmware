#include <string.h>
#include "log.h"
#include "param.h"

#include "dl_general.h"
#include "dl_flowdeck.h"

static logVarId_t id_z;
static logVarId_t id_vx;
static logVarId_t id_vy;
static paramVarId_t id_deck_flow;

void dl_flowdeck_init(void)
{
    id_z  = logGetVarId("range", "zrange");
    id_vx = logGetVarId("motion", "deltaX");
    id_vy = logGetVarId("motion", "deltaY");
    id_deck_flow = paramGetVarId("deck", "bcFlow2");
}

bool dl_flowdeck_read(flow_data_t *data)
{
    if (paramGetUint(id_deck_flow) != 1) {
        return false; // Flowdeck nicht aktiv   
    }

    data->z_mm = (uint16_t)(logGetUint(id_z));      // Höhe in mm
    data->vx = (int16_t)(logGetFloat(id_vx));                  // Bewegung in mm/s (oder px/s je nach Einheit)
    data->vy = (int16_t)(logGetFloat(id_vy));

    return true;
}