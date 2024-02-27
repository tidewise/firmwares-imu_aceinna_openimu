#include "TidewiseMessages.h"
#include "GlobalConstants.h"
#include "boardAPI.h"
#include "platformAPI.h"
#include "sensorsAPI.h"
#include "EKF_Algorithm.h"
#include "WorldMagneticModel.h"

static uint8_t makeFilterFlags() {
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

    return flags;
}

BOOL Fill_IMUStatePacketPayload(uint8_t *payload, uint8_t *payloadLen) {
    imu_state_payload_t *pld = (imu_state_payload_t *)payload;
    *payloadLen = sizeof(imu_state_payload_t);

    pld->filterFlags = makeFilterFlags();
    double temp;
    GetBoardTempData(&temp);
    pld->temperature_C = (uint8_t)temp;
    pld->gpsSolution = gEKFInput.gpsFixType;
    if (gWorldMagModel.validSoln) {
        pld->magneticDeclination = gWorldMagModel.decl_rad;
    }
    else {
        pld->magneticDeclination = 0;
    }

    return TRUE;
}

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

    real geoid2ellipsoid;
    EKF_GetGeoidAboveEllipsoid(&geoid2ellipsoid);
    pld->pos[2] -= geoid2ellipsoid;

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

    EKF_GetCorrectedMags(realData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->magnetometers[i] = (float)realData[i];
    }

    EKF_GetMeasuredEulerAngles(realData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->measuredEulerAngles[i] = (float)realData[i];
    }

    if (EKF_GetMagneticDeclination(realData)) {
        pld->magneticDeclination = (float)realData[0];
    }
    else {
        pld->magneticDeclination = 0;
    }

    pld->filterFlags = makeFilterFlags();
    return TRUE;
}

BOOL Fill_e5PacketPayload(uint8_t *payload, uint8_t *payloadLen) {
    ekf5_payload_t *pld = (ekf5_payload_t *)payload;
    *payloadLen  = sizeof(ekf5_payload_t);
    pld->tstmp   = platformGetIMUCounter();

    real  realData[4];
    double dData[3];

    pld->tstmp   = platformGetIMUCounter();

    EKF_GetEstimatedLLA(dData);
    pld->latitudeRad = dData[0];
    pld->longitudeRad = dData[1];
    pld->altitude = dData[2];

    real geoid2ellipsoid;
    EKF_GetGeoidAboveEllipsoid(&geoid2ellipsoid);
    pld->altitude -= geoid2ellipsoid;

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

    EKF_GetCorrectedMags(realData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->magnetometers[i] = (float)realData[i];
    }

    EKF_GetMeasuredEulerAngles(realData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->measuredEulerAngles[i] = (float)realData[i];
    }

    if (EKF_GetMagneticDeclination(realData)) {
        pld->magneticDeclination = (float)realData[0];
    }
    else {
        pld->magneticDeclination = 0;
    }

    EKF_GetCorrectedAccels(realData);
    for (int i = 0; i < NUM_AXIS; i++) {
        pld->accelerations[i] = (float)realData[i];
    }

    GetBoardTempData(dData);
    pld->temperature_C = dData[0];

    EKF_GetPositionCovariance(pld->covPosition);
    EKF_GetVelocityCovariance(pld->covVelocity);
    EKF_GetQuaternionCovariance(pld->covQuaternion);

    pld->filterFlags = makeFilterFlags();
    return TRUE;
}

