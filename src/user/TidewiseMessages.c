#include "TidewiseMessages.h"
#include "GlobalConstants.h"
#include "platformAPI.h"
#include "EKF_Algorithm.h"

BOOL Fill_e4PacketPayload(uint8_t *payload, uint8_t *payloadLen) {
    ekf4_payload_t *pld = (ekf4_payload_t *)payload;
    *payloadLen  = sizeof(ekf4_payload_t);
    pld->tstmp   = platformGetIMUCounter();

    real  realData[4];
    double dData[3];

    *payloadLen  = sizeof(ekf4_payload_t);
    pld->tstmp   = platformGetIMUCounter();

    EKF_GetEstimatedLLA(dData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->pos[i] = dData[i];
    }

    EKF_GetAttitude_Q(realData);
    for (int i = 0; i < 4; ++i) {
        pld->q[i] = (float)realData[i];
    }

    EKF_GetEstimatedVelocity(realData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->velocity[i] = (float)realData[i];
    }

    EKF_GetCorrectedAngRates(realData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->angularVelocities[i] = (float)realData[i];
    }

    uint8_t opMode, linAccelSw, turnSw;
    EKF_GetOperationalMode(&opMode);
    EKF_GetOperationalSwitches(&linAccelSw, &turnSw);

    uint8_t flags = opMode;
    if (linAccelSw) {
        flags |= 8;
    }
    if (turnSw) {
        flags |= 16;
    }

    pld->flags          = flags;
    return TRUE;
}

