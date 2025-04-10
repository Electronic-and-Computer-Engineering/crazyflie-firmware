#include <stdint.h>
#include "al_filter.h"

static uint16_t buffer[FILTER_LEN];
static uint8_t idx = 0;

uint16_t moving_average(uint16_t new_sample) 
{
    buffer[idx] = new_sample;
    idx = (idx + 1) % FILTER_LEN;

    uint32_t sum = 0;
    for (uint8_t i = 0; i < FILTER_LEN; i++) 
    {
        sum += buffer[i];
    }
    return (uint16_t)(sum / FILTER_LEN);
}