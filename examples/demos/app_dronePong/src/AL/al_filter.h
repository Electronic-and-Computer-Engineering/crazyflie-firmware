#ifndef AL_FILTER_H
#define AL_FILTER_H

#include <stdint.h>
#define FILTER_LEN 4

uint16_t moving_average(uint16_t new_sample);

#endif // AL_FILTER_H