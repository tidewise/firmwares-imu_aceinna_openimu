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
    uint8_t  filterFlags;
    float    q[4];
    float    angularVelocities[3];
    float    velocity[3];
    double   pos[3];
    float    magnetometers[3];
    float    measuredEulerAngles[3];
    float    magneticDeclination;
}ekf4_payload_t;

typedef struct {
    uint32_t tstmp;
    uint8_t  filterFlags;

    float    q[4];
    float    angularVelocities[3];
    float    velocity[3];
    double   latitudeRad;
    double   longitudeRad;
    float    altitude;

    float    magnetometers[3];
    float    measuredEulerAngles[3];
    float    magneticDeclination;

    float    accelerations[3];
    float    covPosition[3];
    float    covVelocity[3];
    float    covQuaternion[10];

    float temperature_C;
}ekf5_payload_t;
#pragma pack()

#define TW_STATE_SIZE 11
#define TW_E4_SIZE sizeof(ekf4_payload_t)
#define TW_E5_SIZE sizeof(ekf5_payload_t)

BOOL Fill_IMUStatePacketPayload(uint8_t *payload, uint8_t *payloadLen);
BOOL Fill_e4PacketPayload(uint8_t *payload, uint8_t *payloadLen);

#endif
