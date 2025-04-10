#ifndef DL_FLOWDECK_H
#define DL_FLOWDECK_H

#include <stdint.h>
#include <stdbool.h>

#include "dl_general.h"

void dl_flowdeck_init(void);
bool dl_flowdeck_read(flow_data_t *data);

#endif // DL_FLOWDECK_H
