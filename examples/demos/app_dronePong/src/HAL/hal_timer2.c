#include "FreeRTOS.h"
#include "hal_timer2.h"
#include "timers.h"

#include "dl_multiranger.h"
#include "dl_flowdeck.h"
#include "dl_imu.h"
#include "dl_general.h"
#include "dl_telemetry.h"

static void cbTimer2(TimerHandle_t xTimer);
static TimerHandle_t timer2 = NULL;

extern sensor_data_t sensorDecks;

void hal_timer2_init(uint32_t period_ms)
{
    if (timer2 == NULL)
    {
        timer2 = xTimerCreate(
            "T2",
            pdMS_TO_TICKS(period_ms),
            pdTRUE,       // periodisch
            NULL,
            cbTimer2
        );
    }

    if (timer2 != NULL)
    {
        xTimerStart(timer2, 0);
    }
}

static void cbTimer2(TimerHandle_t xTimer)
{
    //GET DATA FROM DECKS
    dl_flowdeck_read(&sensorDecks.flow);
    dl_multiranger_read(&sensorDecks.multiranger);
    //dl_imu_read_yaw(&sensorDecks);   
}