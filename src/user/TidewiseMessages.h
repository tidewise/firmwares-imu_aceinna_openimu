#ifndef _TIDEWISE_MESSAGES_H
#define _TIDEWISE_MESSAGES_H

#include <stdint.h>
#include "GlobalConstants.h"

#pragma pack(1)
typedef struct {
    uint32_t tstmp;
    uint8_t  temperature_C;
    uint8_t  gpsSolution;
    float    magneticDeclination;
    uint8_t  filterFlags;
}imu_state_payload_t;

typedef struct {
    uint32_t tstmp;
    float    q[4];
    float    angularVelocities[3];
    float    velocity[3];
    double   pos[3];
    float    magnetometers[3];
    float    measuredEulerAngles[3];
    float    magneticDeclination;
    uint8_t  filterFlags;
}ekf4_payload_t;
#pragma pack()

#define TW_STATE_SIZE 11
#define TW_E4_SIZE 97

BOOL Fill_IMUStatePacketPayload(uint8_t *payload, uint8_t *payloadLen);
BOOL Fill_e4PacketPayload(uint8_t *payload, uint8_t *payloadLen);

#endif
