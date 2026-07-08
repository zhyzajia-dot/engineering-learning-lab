#ifndef RACE_CTRL_H
#define RACE_CTRL_H

#include <stdint.h>

typedef enum {
    RACE_STATE_IDLE = 0,
    RACE_STATE_RUN,
    RACE_STATE_DONE
} RACE_State_t;

typedef struct {
    uint8_t targetLaps;
    uint8_t doneLaps;
    uint8_t cornerCount;
    RACE_State_t state;

    uint8_t sensorMask;
    int16_t lineError;

    int16_t leftTargetMmps;
    int16_t rightTargetMmps;
    int16_t leftSpeedMmps;
    int16_t rightSpeedMmps;
    int32_t leftEncoderCount;
    int32_t rightEncoderCount;

    int16_t leftPwm;
    int16_t rightPwm;
    int32_t distanceMm;

    int16_t avgLeftSpeedMmps;
    int16_t avgRightSpeedMmps;
    int16_t maxSpeedDiffMmps;
    int16_t maxLineTurnPwm;
    uint16_t lineTurnReverseCount;
    uint16_t lineLostEventCount;
    uint8_t centerStablePercent;
    uint16_t sensorMaskChangeCount;
} RACE_Status_t;

void RACE_Init(void);
void RACE_Start(void);
void RACE_Stop(void);
void RACE_Task5ms(void);
uint8_t RACE_IsRunning(void);
void RACE_GetStatus(RACE_Status_t *status);
void RACE_SetTargetLaps(uint8_t laps);
void RACE_NextTargetLap(void);

#endif
