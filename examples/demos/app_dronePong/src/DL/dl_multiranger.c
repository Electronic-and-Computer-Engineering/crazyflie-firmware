#include <string.h>  // Für memset

#include "dl_general.h"
#include "dl_multiranger.h"
#include "log.h"

static logVarId_t idFront, idBack, idLeft, idRight, idUp;
static bool initialized = false;

void dl_multiranger_init(void)
{
    idFront = logGetVarId("range", "front");
    idBack  = logGetVarId("range", "back");
    idLeft  = logGetVarId("range", "left");
    idRight = logGetVarId("range", "right");
    idUp    = logGetVarId("range", "up");

    initialized = (idFront >= 0 && idBack >= 0 && idLeft >= 0 && idRight >= 0 && idUp >= 0);
}

bool dl_multiranger_read(multiranger_data_t* data)
{
    if (!initialized || data == NULL) {
        return false;
    }

    data->front = (uint16_t)(logGetUint(idFront));
    data->back  = (uint16_t)(logGetUint(idBack));
    data->left  = (uint16_t)(logGetUint(idLeft));
    data->right = (uint16_t)(logGetUint(idRight));
    data->up    = (uint16_t)(logGetUint(idUp));

    return true;
}