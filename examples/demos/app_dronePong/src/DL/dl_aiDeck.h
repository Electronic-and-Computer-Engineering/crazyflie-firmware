#ifndef DL_AIDECK_H
#define DL_AIDECK_H

#include "cpx.h"
#include "cpx_internal_router.h"

void cpxPacketCallback(const CPXPacket_t* cpxRx);
void aiDeck_init(void);
void aiDeck_request(void);

#endif