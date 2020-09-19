#ifndef _TIDEWISE_MESSAGES_H
#define _TIDEWISE_MESSAGES_H

#include <stdint.h>
#include "GlobalConstants.h"

#pragma pack(1)
typedef struct {
    uint32_t tstmp;
    float    q[4];
    float    angularVelocities[3];
    float    velocity[3];
    double   pos[3];
    uint8_t  flags;
}ekf4_payload_t;
#pragma pack()

BOOL Fill_e4PacketPayload(uint8_t *payload, uint8_t *payloadLen);

#endif
