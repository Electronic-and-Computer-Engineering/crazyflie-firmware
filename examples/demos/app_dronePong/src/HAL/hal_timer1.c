#include <stdint.h>
#include "FreeRTOS.h"
#include "hal_timer1.h"
#include "timers.h"

#include "dl_general.h"
#include "dl_telemetry.h"

static void cbTimer1(TimerHandle_t xTimer);
static TimerHandle_t timer1 = NULL;

extern sensor_data_t   sensorDecks;

void hal_timer1_init(uint32_t period_ms)
{
    if (timer1 == NULL)
    {
        timer1 = xTimerCreate(
            "T1",
            pdMS_TO_TICKS(period_ms),
            pdTRUE,       // periodisch
            NULL,
            cbTimer1
        );
    }

    if (timer1 != NULL)
    {
        xTimerStart(timer1, 0);
    }
}

static void cbTimer1(TimerHandle_t xTimer)
{
    telemetrySendMode(&sensorDecks);
}