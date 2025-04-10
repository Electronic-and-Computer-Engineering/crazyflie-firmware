#ifndef DL_MULTIRANGER_H
#define DL_MULTIRANGER_H

#include <stdint.h>
#include <stdbool.h>

#include "dl_general.h"

void dl_multiranger_init(void);
bool dl_multiranger_read(multiranger_data_t *data);

#endif // DL_MULTIRANGER_H
