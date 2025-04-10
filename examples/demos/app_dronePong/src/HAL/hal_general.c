#include "hal_general.h"
#include "hal_timer1.h"
#include "hal_timer2.h"

void HAL_init(void) 
{
    hal_timer1_init(200);
    hal_timer2_init(10);
}