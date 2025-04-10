#define DEBUG_MODULE "UWBDeck"
#include "debug.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "stm32fxxx.h"
#include "config.h"
#include "console.h"
#include "uart2.h"
#include "deck.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "log.h"
#include "param.h"

#include "deck.h"

void uwbdeckTask(void *param){
  uint8_t c;

  while(1){
    uart2Getchar(&c);
    consolePutchar(c);
  }
}


static void uwbdeckInit()
{
  DEBUG_PRINT("Initializing UWB Deck\n");
  uart2Init(115200);

  xTaskCreate(uwbdeckTask, "UWBDECK", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

static bool uwbdeckTest()
{
  return true;
}

static const DeckDriver uwbDriver = {
  .usedGpio = 0,
  //.usedPeriph = DECK_USING_UART1,
  .usedPeriph = DECK_USING_UART2,
  .vid = 0x44,
  .pid = 0x80,
  .name = "uwbdeck",
  .init = uwbdeckInit,
  .test = uwbdeckTest,
};

DECK_DRIVER(uwbDriver);
