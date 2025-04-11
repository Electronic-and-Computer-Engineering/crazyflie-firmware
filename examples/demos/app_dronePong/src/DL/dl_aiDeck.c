#include "dl_aiDeck.h"
#include "dl_general.h"
#include "FreeRTOS.h"

extern sensor_data_t   sensorDecks;

void cpxPacketCallback(const CPXPacket_t* cpxRx) {
    sensorDecks.patchDetected = cpxRx->data[0];
}

void aiDeck_init(void)
{
    cpxInit();
    cpxRegisterAppMessageHandler(cpxPacketCallback);   
}

void aiDeck_request(void)
{
    CPXPacket_t packet;
    packet.data[0] = 0x00;
    packet.dataLength = 1;

    cpxInitRoute(CPX_T_STM32, CPX_T_GAP8, CPX_F_APP, &packet.route);
    cpxSendPacketBlocking(&packet);
}
