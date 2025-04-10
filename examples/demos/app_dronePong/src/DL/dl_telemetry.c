#include "dl_telemetry.h"
#include "dl_general.h"
#include "crtp.h"

#include "supervisor.h"

extern sensor_data_t   sensorDecks;
extern State state;

void telemetrySendMode(const sensor_data_t *sensor)
{
    CRTPPacket pk;
    pk.header = CRTP_HEADER(TELEMETRY_PORT, TELEMETRY_CHANNEL);

    uint8_t i = 0;
    write_u16(pk.data, &i, sensor->droneFlags);
    write_u16(pk.data, &i, sensor->multiranger.front);
    write_u16(pk.data, &i, sensor->multiranger.back);
    write_u16(pk.data, &i, sensor->multiranger.left);
    write_u16(pk.data, &i, sensor->multiranger.right);
    write_u16(pk.data, &i, sensor->multiranger.up);
    write_u16(pk.data, &i, sensor->flow.z_mm);
    write_u16(pk.data, &i, sensor->imuData.iscaledYaw);

    pk.size = i;
    crtpSendPacket(&pk);
}

void write_u16(uint8_t *dst, uint8_t *idx, uint16_t value) 
{
    dst[(*idx)++] = (uint8_t)(value >> 8);
    dst[(*idx)++] = (uint8_t)(value & 0xFF);
}

void write_i16(uint8_t *dst, uint8_t *idx, int16_t value)
{
    uint16_t uvalue = (uint16_t)value;
    dst[(*idx)++] = (uint8_t)(uvalue >> 8);
    dst[(*idx)++] = (uint8_t)(uvalue & 0xFF);
}

static void commandPacketHandler(CRTPPacket *pk)
{
    if ((pk->size >= 1)) {
        uint8_t commandId = pk->data[0];
        switch (commandId) 
        {
            case 0x01:
                state = ARM;               
                break;
            case 0x02:
                state = UNARM;                
                break;
            case 0x04:
                state = HOVER; 
                break;
            case 0x08:
                state = LAND; 
                break;
            case 0x10:
                state = DIRECTIONAL; 
                break;
            case 0x12:
                state = ROTATE0; 
                break;                 
            default:
                break;
        }
    }
}

void telemetryRecMode(void)
{
    crtpRegisterPortCB(TELEMETRY_PORT, commandPacketHandler);
}

