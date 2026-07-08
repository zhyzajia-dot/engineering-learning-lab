#include <stdint.h>
#include "race_ctrl.h"
#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "sensor.h"
#include "ti_msp_dl_config.h"

/* Race control: line following, speed loop and left-corner handling. */

/* Tunable parameters */

#define TARGET_SPEED_MMPS               330
#define TURN_SPEED_MMPS                 160

#define PWM_MAX_OUT                     620
#define PWM_FF_NUM                      34
#define PWM_FF_DIV                      100

#define MIN_RUN_PWM_LEFT                45
#define MIN_RUN_PWM_RIGHT               45

#define SPEED_KP_NUM                    58
#define SPEED_KP_DIV                    100
#define SPEED_KI_NUM                    0
#define SPEED_KI_DIV                    100
#define SPEED_I_LIMIT                   12000

/* Sensor direction */
#define LINE_TURN_SIGN                  1
#define SENSOR_REVERSE_ORDER            0
#define SENSOR_BLACK_IS_1               1
#define SENSOR_8_MASK                   (1U << 7)

/* Line-following PD */
#define LINE_KP_NUM                     650
#define LINE_KP_DIV                     100

#define LINE_KD_AWAY_NUM                185
#define LINE_KD_RETURN_NUM              980
#define LINE_KD_DIV                     100
#define LINE_D_AWAY_LIMIT_PWM            25
#define LINE_D_RETURN_LIMIT_PWM         108
#define LINE_START_D_PRELOAD_DIV          4

#define STRAIGHT_TRIM_PWM                0
#define LINE_TURN_LIMIT                 150
#define LINE_TURN_SPEED_NUM               3
#define LINE_TURN_SPEED_DIV               2
#define LINE_MIN_INNER_SPEED_MMPS        20
#define LINE_WHEEL_DIFF_DAMP_NUM          1
#define LINE_WHEEL_DIFF_DAMP_DIV          2

#define LINE_LOST_HOLD_MS               50U
#define LINE_START_CONFIRM_COUNT        1U
#define LINE_RECOVERY_SPEED_MMPS       140

/* Left corner */
#define LEFT_TURN_APPROACH_PWM            140
#define LEFT_TURN_APPROACH_MS              40U
#define LEFT_TURN_LEFT_PWM_FAST        (-185)
#define LEFT_TURN_RIGHT_PWM_FAST         185
#define LEFT_TURN_LEFT_PWM_FINAL       (-140)
#define LEFT_TURN_RIGHT_PWM_FINAL        140
#define LEFT_TURN_TARGET_YAW_X10         900
#define LEFT_TURN_FINAL_START_YAW_X10   (LEFT_TURN_TARGET_YAW_X10 - 180)
#define LEFT_TURN_MIN_MS                240U
#define LEFT_TURN_OPEN_LOOP_FAST_MS     650U
#define LEFT_TURN_OPEN_LOOP_MS          850U
#define LEFT_TURN_MAX_MS               1200U
#define LEFT_TURN_LINE_GUARD_YAW_X10    LEFT_TURN_TARGET_YAW_X10
#define LEFT_TURN_LINE_FALLBACK_MS       850U
#define LEFT_TURN_LINE_CLEAR_COUNT         2U
#define LEFT_TURN_LINE_CAPTURE_COUNT       3U
#define LEFT_TURN_EXIT_MS              120U
#define LEFT_TURN_EXIT_MAX_MS          600U
#define LEFT_TURN_EXIT_LINE_COUNT         3U
#define LEFT_TURN_EXIT_SPEED_MMPS       180
#define LEFT_TURN_EXIT_MIN_PWM          110
#define LEFT_TURN_EXIT_LIMIT_PWM         45
#define LEFT_CORNER_ARM_CONFIRM_COUNT     8U
#define LEFT_CORNER_ARM_ERROR_LIMIT      11
#define LEFT_CORNER_LATCH_MS             80U

/* Mixing and safety */
#define STRAIGHT_USE_ENCODER_SPEED_LOOP 1
#define SPEED_FB_LIMIT_PWM             165
#define SPEED_BRAKE_LIMIT_PWM          170
#define LINE_CENTER_DEADBAND              0
#define LINE_LOST_STEER_HOLD_MS          15U

/* Internal state */

typedef enum {
    MODE_LINE = 0,
    MODE_LEFT_TURN,
    MODE_LEFT_TURN_EXIT
} RaceMode_t;

static RACE_Status_t g_status;
static RaceMode_t g_mode = MODE_LINE;
static uint32_t g_turnMs = 0U;
static uint8_t g_exitLineConfirm = 0U;
static uint8_t g_turnApproachDone = 0U;
static uint8_t g_turnYawStarted = 0U;
static uint8_t g_turnLineCleared = 0U;
static uint8_t g_turnLineClearConfirm = 0U;
static uint8_t g_turnLineCaptureConfirm = 0U;
static uint8_t g_leftCornerArmed = 0U;
static uint8_t g_leftCornerArmConfirm = 0U;
static uint8_t g_leftCornerLatchMs = 0U;
static uint8_t g_lapCompletePending = 0U;
static int16_t g_leftSpeedFilt = 0;
static int16_t g_rightSpeedFilt = 0;
static int32_t g_leftSpeedI = 0;
static int32_t g_rightSpeedI = 0;
static int16_t g_leftOut = 0;
static int16_t g_rightOut = 0;
static int16_t g_lineRawErr = 0;
static int16_t g_lineErrFilt = 0;
static int16_t g_lineTurnNow = 0;
static uint32_t g_lineLostMs = 0U;
static uint8_t g_lineValidNow = 0U;
static uint8_t g_lineSeenConfirm = 0U;
static uint8_t g_lineDriveUnlocked = 0U;
static uint32_t g_diagLineSamples = 0U;
static int32_t g_diagLeftSpeedSum = 0;
static int32_t g_diagRightSpeedSum = 0;
static uint32_t g_diagCenterStableSamples = 0U;
static int8_t g_diagLastTurnSign = 0;
static uint8_t g_diagWasLineValid = 0U;
static uint8_t g_diagLastSensorMask = 0U;

/* Helpers */

static int16_t clamp_i16(int16_t v, int16_t minV, int16_t maxV)
{
    if (v < minV) return minV;
    if (v > maxV) return maxV;
    return v;
}

static int32_t clamp_i32(int32_t v, int32_t minV, int32_t maxV)
{
    if (v < minV) return minV;
    if (v > maxV) return maxV;
    return v;
}

static int16_t abs_i16(int16_t v)
{
    return (v >= 0) ? v : (int16_t)(-v);
}

static void reset_line_steering(void)
{
    g_lineRawErr = 0;
    g_lineErrFilt = 0;
    g_lineTurnNow = 0;
}

static void reset_straight_speed(void)
{
    g_leftSpeedI = 0;
    g_rightSpeedI = 0;
    g_lineDriveUnlocked = 0U;
}

static uint32_t irq_save(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void irq_restore(uint32_t primask)
{
    if (primask == 0U) __enable_irq();
}

static uint8_t center_line_exact(uint8_t sensorMask);

static void reset_line_diag(void)
{
    g_diagLineSamples = 0U;
    g_diagLeftSpeedSum = 0;
    g_diagRightSpeedSum = 0;
    g_diagCenterStableSamples = 0U;
    g_diagLastTurnSign = 0;
    g_diagWasLineValid = 0U;
    g_diagLastSensorMask = 0U;

    g_status.avgLeftSpeedMmps = 0;
    g_status.avgRightSpeedMmps = 0;
    g_status.maxSpeedDiffMmps = 0;
    g_status.maxLineTurnPwm = 0;
    g_status.lineTurnReverseCount = 0U;
    g_status.lineLostEventCount = 0U;
    g_status.centerStablePercent = 0U;
    g_status.sensorMaskChangeCount = 0U;
}

static void update_line_diag(int16_t leftSpeed, int16_t rightSpeed,
                             int16_t lineTurnPwm, uint8_t sensorMask)
{
    int16_t speedDiff = abs_i16((int16_t)(leftSpeed - rightSpeed));
    int16_t turnAbs = abs_i16(lineTurnPwm);
    int8_t turnSign = 0;

    if (lineTurnPwm >= 8) {
        turnSign = 1;
    } else if (lineTurnPwm <= -8) {
        turnSign = -1;
    }

    if (g_lineValidNow != 0U) {
        if (g_diagLineSamples < 500000U) {
            g_diagLineSamples++;
            g_diagLeftSpeedSum += leftSpeed;
            g_diagRightSpeedSum += rightSpeed;

            if (center_line_exact(sensorMask) != 0U) {
                g_diagCenterStableSamples++;
            }
        }

        if (g_diagLineSamples != 0U) {
            g_status.avgLeftSpeedMmps =
                (int16_t)(g_diagLeftSpeedSum / (int32_t)g_diagLineSamples);
            g_status.avgRightSpeedMmps =
                (int16_t)(g_diagRightSpeedSum / (int32_t)g_diagLineSamples);
            g_status.centerStablePercent =
                (uint8_t)((g_diagCenterStableSamples * 100U) /
                          g_diagLineSamples);
        }

        if (speedDiff > g_status.maxSpeedDiffMmps) {
            g_status.maxSpeedDiffMmps = speedDiff;
        }

        if (turnAbs > g_status.maxLineTurnPwm) {
            g_status.maxLineTurnPwm = turnAbs;
        }

        if (turnSign != 0) {
            if ((g_diagLastTurnSign != 0) && (turnSign != g_diagLastTurnSign)) {
                if (g_status.lineTurnReverseCount < 65535U) {
                    g_status.lineTurnReverseCount++;
                }
            }
            g_diagLastTurnSign = turnSign;
        }

        if ((g_diagLastSensorMask != 0U) &&
            (sensorMask != g_diagLastSensorMask) &&
            (g_status.sensorMaskChangeCount < 65535U)) {
            g_status.sensorMaskChangeCount++;
        }
        g_diagLastSensorMask = sensorMask;
    }

    if ((g_diagWasLineValid != 0U) && (g_lineValidNow == 0U)) {
        if (g_status.lineLostEventCount < 65535U) {
            g_status.lineLostEventCount++;
        }
    }
    g_diagWasLineValid = g_lineValidNow;
}

/* Sensor handling */

static uint8_t sensor_bit_active(uint8_t sensorMask, uint8_t index)
{
    uint8_t bitIndex = SENSOR_REVERSE_ORDER ? (uint8_t)(7U - index) : index;
    uint8_t bit = (uint8_t)((sensorMask >> bitIndex) & 0x01U);
    return SENSOR_BLACK_IS_1 ? bit : (bit == 0U) ? 1U : 0U;
}

static uint8_t center_line_exact(uint8_t sensorMask)
{
    uint8_t i;

    if ((sensor_bit_active(sensorMask, 3U) == 0U) ||
        (sensor_bit_active(sensorMask, 4U) == 0U)) {
        return 0U;
    }

    for (i = 0U; i < 8U; i++) {
        if ((i != 3U) && (i != 4U) &&
            (sensor_bit_active(sensorMask, i) != 0U)) {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t turn_center_line_seen(uint8_t sensorMask)
{
    if ((SENSOR_GetI2cOk() == 0U) || (sensorMask == 0U)) {
        return 0U;
    }

    return ((sensor_bit_active(sensorMask, 3U) != 0U) ||
            (sensor_bit_active(sensorMask, 4U) != 0U)) ? 1U : 0U;
}

static int16_t calc_line_error(uint8_t sensorMask, uint8_t *valid) 
{ 
    static const int16_t weights[8] = {-36, -25, -11, -2, 2, 11, 25, 36}; 
    int32_t sum = 0; 
    uint8_t count = 0U; 
    uint8_t i; 
    uint8_t leftOuter = 0U;
    uint8_t rightOuter = 0U;
    uint8_t center = 0U;
     
    *valid = 0U; 
    if (sensorMask == 0U) { 
        return 0; 
    } 
     
    for (i = 0U; i < 8U; i++) { 
        if (sensor_bit_active(sensorMask, i) != 0U) { 
            sum += weights[i]; 
            count++; 
            if (i <= 2U) leftOuter = 1U;
            if (i >= 5U) rightOuter = 1U;
            if ((i == 3U) || (i == 4U)) center = 1U;
        } 
    } 
     
    if (count == 0U || count >= 5U) {
        return 0;
    }

    if ((center == 0U) && (leftOuter != 0U) && (rightOuter != 0U)) {
        return 0;
    }
    
    *valid = 1U; 
    return (int16_t)(sum / count); 
}

static uint8_t left_corner_present(uint8_t sensorMask)
{
    uint8_t s0 = sensor_bit_active(sensorMask, 0U);
    uint8_t s1 = sensor_bit_active(sensorMask, 1U);
    uint8_t s2 = sensor_bit_active(sensorMask, 2U);
    uint8_t s3 = sensor_bit_active(sensorMask, 3U);
    uint8_t s6 = sensor_bit_active(sensorMask, 6U);
    uint8_t s7 = sensor_bit_active(sensorMask, 7U);
    uint8_t leftCount = (uint8_t)(s0 + s1 + s2 + s3);

    if ((SENSOR_GetI2cOk() == 0U) || (sensorMask == 0U)) {
        return 0U;
    }

    if ((leftCount >= 3U) && (s6 == 0U) && (s7 == 0U)) {
        return 1U;
    }

    return 0U;
}

static uint8_t stable_line_for_corner_arm(uint8_t sensorMask)
{
    uint8_t s0 = sensor_bit_active(sensorMask, 0U);
    uint8_t s1 = sensor_bit_active(sensorMask, 1U);
    uint8_t s6 = sensor_bit_active(sensorMask, 6U);
    uint8_t s7 = sensor_bit_active(sensorMask, 7U);

    if ((SENSOR_GetI2cOk() == 0U) ||
        (g_lineValidNow == 0U) ||
        (abs_i16(g_lineErrFilt) > LEFT_CORNER_ARM_ERROR_LIMIT)) {
        return 0U;
    }

    if ((s0 != 0U) || (s1 != 0U) || (s6 != 0U) || (s7 != 0U)) {
        return 0U;
    }

    return 1U;
}

static uint8_t update_left_corner_detector(uint8_t sensorMask)
{
    uint8_t s0 = sensor_bit_active(sensorMask, 0U);
    uint8_t s1 = sensor_bit_active(sensorMask, 1U);
    uint8_t s2 = sensor_bit_active(sensorMask, 2U);
    uint8_t s6 = sensor_bit_active(sensorMask, 6U);
    uint8_t s7 = sensor_bit_active(sensorMask, 7U);

    if (g_leftCornerArmed == 0U) {
        g_leftCornerLatchMs = 0U;

        if (stable_line_for_corner_arm(sensorMask) != 0U) {
            if (g_leftCornerArmConfirm < 255U) {
                g_leftCornerArmConfirm++;
            }
        } else {
            g_leftCornerArmConfirm = 0U;
        }

        if (g_leftCornerArmConfirm >= LEFT_CORNER_ARM_CONFIRM_COUNT) {
            g_leftCornerArmed = 1U;
            g_leftCornerArmConfirm = 0U;
        }

        return 0U;
    }

    if (left_corner_present(sensorMask) != 0U) {
        g_leftCornerArmed = 0U;
        g_leftCornerArmConfirm = 0U;
        g_leftCornerLatchMs = 0U;
        return 1U;
    }

    if ((s1 != 0U) && (s2 != 0U) && (s6 == 0U) && (s7 == 0U)) {
        g_leftCornerLatchMs = LEFT_CORNER_LATCH_MS;
    } else if (g_leftCornerLatchMs >= 5U) {
        g_leftCornerLatchMs = (uint8_t)(g_leftCornerLatchMs - 5U);
    } else {
        g_leftCornerLatchMs = 0U;
    }

    if ((g_leftCornerLatchMs != 0U) &&
        (s0 != 0U) &&
        (s6 == 0U) &&
        (s7 == 0U)) {
        g_leftCornerArmed = 0U;
        g_leftCornerArmConfirm = 0U;
        g_leftCornerLatchMs = 0U;
        return 1U;
    }

    return 0U;
}

/* Speed loop */

static int16_t calc_speed_pwm(int16_t targetSpeed, int16_t realSpeed, int32_t *integral, int16_t minPwm) 
{
    int16_t err;
    int32_t ff;
    int32_t feedback;
    int32_t out;

    if (targetSpeed <= 0) {
        if (integral != 0) *integral = 0;
        return 0;
    }

    err = (int16_t)(targetSpeed - realSpeed);

    if ((SPEED_KI_NUM != 0) && (integral != 0)) {
        if (abs_i16(err) < 150) {
            *integral += err;
            *integral = clamp_i32(*integral,
                                  (int32_t)-SPEED_I_LIMIT,
                                  (int32_t)SPEED_I_LIMIT);
        }
    }

    ff = (int32_t)minPwm +
         (((int32_t)targetSpeed * PWM_FF_NUM) / PWM_FF_DIV);
    feedback = (((int32_t)SPEED_KP_NUM * err) / SPEED_KP_DIV);
    feedback = clamp_i32(feedback,
                         (int32_t)-SPEED_FB_LIMIT_PWM,
                         (int32_t)SPEED_FB_LIMIT_PWM);
    out = ff + feedback;

    if ((SPEED_KI_NUM != 0) && (integral != 0)) {
        out += (((int32_t)SPEED_KI_NUM * (*integral)) / SPEED_KI_DIV);
    }

    out = clamp_i32(out,
                    (int32_t)-SPEED_BRAKE_LIMIT_PWM,
                    (int32_t)PWM_MAX_OUT);
    return (int16_t)out;
}

static int16_t calc_feedforward_pwm(int16_t targetSpeed, int16_t minPwm)
{
    int32_t out;

    if (targetSpeed <= 0) {
        return 0;
    }

    out = (int32_t)minPwm +
          (((int32_t)targetSpeed * PWM_FF_NUM) / PWM_FF_DIV);
    out = clamp_i32(out, 0, PWM_MAX_OUT);
    return (int16_t)out;
}

/* Line correction */

static int16_t calc_line_turn(uint8_t sensorMask)
{
    uint8_t valid;
    int16_t rawErr = calc_line_error(sensorMask, &valid);

    if (valid != 0U) {
        int16_t ctrlErr = rawErr;
        int16_t desiredTurn;
        int16_t previousError = g_lineErrFilt;
        int16_t errDelta;
        int32_t dGain;
        int32_t dLimit;
        int32_t dTerm;
        int32_t pTerm;

        g_lineValidNow = 1U;
        if (g_lineSeenConfirm < 255U) {
            g_lineSeenConfirm++;
        }
        g_lineLostMs = 0U;

        if (g_lineSeenConfirm <= 1U) {
            previousError = ctrlErr;
        }

        g_lineRawErr = rawErr;
        g_lineErrFilt = ctrlErr;

        if (abs_i16(g_lineErrFilt) <= LINE_CENTER_DEADBAND) {
            pTerm = 0;
        } else {
            pTerm = ((int32_t)LINE_KP_NUM * g_lineErrFilt) / LINE_KP_DIV;
        }

        errDelta = (int16_t)(g_lineErrFilt - previousError);
        if (abs_i16(g_lineErrFilt) < abs_i16(previousError)) {
            dGain = LINE_KD_RETURN_NUM;
            dLimit = LINE_D_RETURN_LIMIT_PWM;
        } else {
            dGain = LINE_KD_AWAY_NUM;
            dLimit = LINE_D_AWAY_LIMIT_PWM;
        }

        dTerm = (dGain * errDelta) / LINE_KD_DIV;
        dTerm = clamp_i32(dTerm,
                          -dLimit,
                          dLimit);
        if ((g_lineErrFilt == 0) && (previousError == 0)) {
            dTerm = 0;
        }

        desiredTurn = (int16_t)(pTerm + dTerm);
        desiredTurn = (int16_t)(desiredTurn * LINE_TURN_SIGN);
        desiredTurn = clamp_i16(desiredTurn,
                                (int16_t)-LINE_TURN_LIMIT,
                                LINE_TURN_LIMIT);
        g_lineTurnNow = desiredTurn;
    } else {
        g_lineValidNow = 0U;
        g_lineSeenConfirm = 0U;
        g_lineLostMs += 5U;

        if (g_lineLostMs > LINE_LOST_STEER_HOLD_MS) {
            reset_line_steering();
        }
    }
    
    return g_lineTurnNow;
}

/* Corner handling */

static int16_t left_turn_yaw_progress_x10(void)
{
    int16_t yaw = IMU_GetRelativeYawX10();

    if (yaw < 0) {
        yaw = (int16_t)(-yaw);
    }

    return yaw;
}

static void enter_left_turn(void)
{
    g_mode = MODE_LEFT_TURN;
    g_turnMs = 0U;
    g_exitLineConfirm = 0U;
    g_turnApproachDone = 0U;
    g_turnYawStarted = 0U;
    g_turnLineCleared = 0U;
    g_turnLineClearConfirm = 0U;
    g_turnLineCaptureConfirm = 0U;
    g_leftCornerArmed = 0U;
    g_leftCornerArmConfirm = 0U;
    g_leftCornerLatchMs = 0U;
    g_lapCompletePending = 0U;
    reset_straight_speed();
    reset_line_steering();
    g_leftOut = LEFT_TURN_APPROACH_PWM;
    g_rightOut = LEFT_TURN_APPROACH_PWM;
    MOTOR_SetPWM(g_leftOut, g_rightOut);
    g_diagLastTurnSign = 0;
    g_diagWasLineValid = 0U;
    g_diagLastSensorMask = 0U;
}

static void finish_left_turn(void)
{
    g_mode = MODE_LEFT_TURN_EXIT;
    g_turnMs = 0U;
    g_exitLineConfirm = 0U;
    g_leftCornerArmed = 0U;
    g_leftCornerArmConfirm = 0U;
    g_leftCornerLatchMs = 0U;
    g_lapCompletePending = 0U;
    
    g_status.cornerCount++;
    if (g_status.cornerCount >= 4U) {
        g_lapCompletePending = 1U;
    }
    
    reset_line_steering();
    g_leftSpeedI = 0;
    g_rightSpeedI = 0;

    if (g_status.state != RACE_STATE_RUN) {
        g_leftOut = 0;
        g_rightOut = 0;
        g_status.leftPwm = 0;
        g_status.rightPwm = 0;
        return;
    }

    g_leftOut = calc_feedforward_pwm(LEFT_TURN_EXIT_SPEED_MMPS,
                                     MIN_RUN_PWM_LEFT);
    if (g_leftOut < LEFT_TURN_EXIT_MIN_PWM) {
        g_leftOut = LEFT_TURN_EXIT_MIN_PWM;
    }
    g_rightOut = g_leftOut;
    MOTOR_SetPWM(g_leftOut, g_rightOut);
}

static uint8_t complete_pending_lap(void)
{
    if (g_lapCompletePending != 0U) {
        g_lapCompletePending = 0U;
        g_status.cornerCount = 0U;
        if (g_status.doneLaps < g_status.targetLaps) {
            g_status.doneLaps++;
        }

        if (g_status.doneLaps >= g_status.targetLaps) {
            g_status.state = RACE_STATE_DONE;
            MOTOR_Stop();
            g_leftOut = 0;
            g_rightOut = 0;
            g_status.leftPwm = 0;
            g_status.rightPwm = 0;
            return 1U;
        }
    }

    return 0U;
}

static uint8_t finish_left_turn_exit(void)
{
    g_mode = MODE_LINE;
    g_turnMs = 0U;
    g_exitLineConfirm = 0U;
    g_leftSpeedI = 0;
    g_rightSpeedI = 0;
    g_lineDriveUnlocked = 1U;
    g_leftCornerArmed = 0U;
    g_leftCornerArmConfirm = 0U;
    g_leftCornerLatchMs = 0U;

    return complete_pending_lap();
}

/* Public API */

void RACE_Init(void)
{
    g_status.targetLaps = 1U;
    g_status.doneLaps = 0U;
    g_status.cornerCount = 0U;
    g_status.state = RACE_STATE_IDLE;
    g_status.sensorMask = 0U;
    g_status.lineError = 0;
    g_status.leftTargetMmps = 0;
    g_status.rightTargetMmps = 0;
    g_status.leftSpeedMmps = 0;
    g_status.rightSpeedMmps = 0;
    g_status.leftEncoderCount = 0;
    g_status.rightEncoderCount = 0;
    g_status.leftPwm = 0;
    g_status.rightPwm = 0;
    g_status.distanceMm = 0;
    reset_line_diag();
    
    g_mode = MODE_LINE;
    g_turnMs = 0U;
    g_exitLineConfirm = 0U;
    g_leftCornerArmed = 0U;
    g_leftCornerArmConfirm = 0U;
    g_leftCornerLatchMs = 0U;
    g_lapCompletePending = 0U;
    g_leftSpeedFilt = 0;
    g_rightSpeedFilt = 0;
    reset_straight_speed();
    g_leftOut = 0;
    g_rightOut = 0;
    reset_line_steering();
    
    g_lineLostMs = 1000U;
    g_lineValidNow = 0U;
    g_lineSeenConfirm = 0U;
    MOTOR_Stop();
}

void RACE_Start(void)
{
    ENCODER_Reset();
    
    g_status.state = RACE_STATE_RUN;
    g_status.doneLaps = 0U;
    g_status.cornerCount = 0U;
    g_status.distanceMm = 0;
    g_status.leftEncoderCount = 0;
    g_status.rightEncoderCount = 0;
    g_status.lineError = 0;
    reset_line_diag();
    
    g_mode = MODE_LINE;
    g_turnMs = 0U;
    g_exitLineConfirm = 0U;
    g_leftCornerArmed = 0U;
    g_leftCornerArmConfirm = 0U;
    g_leftCornerLatchMs = 0U;
    g_lapCompletePending = 0U;
    g_leftSpeedFilt = 0;
    g_rightSpeedFilt = 0;
    reset_straight_speed();
    g_leftOut = 0;
    g_rightOut = 0;
    reset_line_steering();
    
    g_lineLostMs = 1000U;
    g_lineValidNow = 0U;
    g_lineSeenConfirm = 0U;
    
    MOTOR_SetPWM(0, 0);
}

void RACE_Stop(void)
{
    g_status.state = RACE_STATE_IDLE;
    reset_straight_speed();
    g_leftOut = 0;
    g_rightOut = 0;
    reset_line_steering();
    g_lineValidNow = 0U;
    g_lineSeenConfirm = 0U;
    g_mode = MODE_LINE;
    g_turnMs = 0U;
    g_exitLineConfirm = 0U;
    g_leftCornerArmed = 0U;
    g_leftCornerArmConfirm = 0U;
    g_leftCornerLatchMs = 0U;
    g_lapCompletePending = 0U;
    
    MOTOR_Stop();
    
    g_status.leftTargetMmps = 0;
    g_status.rightTargetMmps = 0;
    g_status.leftPwm = 0;
    g_status.rightPwm = 0;
}

uint8_t RACE_IsRunning(void)
{
    return (g_status.state == RACE_STATE_RUN) ? 1U : 0U;
}

void RACE_Task5ms(void)
{
    uint8_t sensorMask =
        (uint8_t)(SENSOR_GetRawMask() & (uint8_t)(~SENSOR_8_MASK));
    int16_t leftSpeed = ENCODER_GetLeftSpeed();
    int16_t rightSpeed = ENCODER_GetRightSpeed();
    int32_t leftCount = ENCODER_GetLeftCount();
    int32_t rightCount = ENCODER_GetRightCount();
    int32_t leftDist = ENCODER_GetLeftDistanceMm();
    int32_t rightDist = ENCODER_GetRightDistanceMm();
    int32_t avgDist = (leftDist + rightDist) / 2;
    
    g_status.sensorMask = sensorMask;
    g_status.distanceMm = avgDist;
    g_status.leftSpeedMmps = leftSpeed;
    g_status.rightSpeedMmps = rightSpeed;

    if (g_status.state == RACE_STATE_RUN) {
        g_status.leftEncoderCount = leftCount;
        g_status.rightEncoderCount = rightCount;
    }
    
    if (g_status.state != RACE_STATE_RUN) {
        MOTOR_Stop();
        g_leftOut = 0;
        g_rightOut = 0;
        reset_straight_speed();
        g_status.leftTargetMmps = 0;
        g_status.rightTargetMmps = 0;
        g_status.leftPwm = 0;
        g_status.rightPwm = 0;
        return;
    }

    if (g_mode == MODE_LEFT_TURN) {
        int16_t yawProgress = 0;
        uint8_t centerSeen = turn_center_line_seen(sensorMask);

        if (g_turnApproachDone == 0U) {
            g_turnMs += 5U;
            g_leftOut = LEFT_TURN_APPROACH_PWM;
            g_rightOut = LEFT_TURN_APPROACH_PWM;

            if (g_turnMs >= LEFT_TURN_APPROACH_MS) {
                g_turnApproachDone = 1U;
                g_turnMs = 0U;
                if (IMU_IsReady() != 0U) {
                    IMU_BeginRelativeYaw();
                    g_turnYawStarted = 1U;
                }
                g_leftOut = LEFT_TURN_LEFT_PWM_FAST;
                g_rightOut = LEFT_TURN_RIGHT_PWM_FAST;
            }

            MOTOR_SetPWM(g_leftOut, g_rightOut);
            g_status.leftTargetMmps = TURN_SPEED_MMPS;
            g_status.rightTargetMmps = TURN_SPEED_MMPS;
            g_status.leftPwm = g_leftOut;
            g_status.rightPwm = g_rightOut;
            g_status.lineError = 0;
            return;
        }

        g_turnMs += 5U;

        if (g_turnYawStarted == 0U) {
            if (IMU_IsReady() != 0U) {
                IMU_BeginRelativeYaw();
                g_turnYawStarted = 1U;
            }
        } else if (IMU_IsReady() != 0U) {
            yawProgress = left_turn_yaw_progress_x10();
        }

        if (g_turnLineCleared == 0U) {
            if (centerSeen == 0U) {
                if (g_turnLineClearConfirm < 255U) {
                    g_turnLineClearConfirm++;
                }
            } else {
                g_turnLineClearConfirm = 0U;
            }

            if (g_turnLineClearConfirm >= LEFT_TURN_LINE_CLEAR_COUNT) {
                g_turnLineCleared = 1U;
                g_turnLineCaptureConfirm = 0U;
            }
        } else {
            uint8_t lineCaptureAllowed =
                ((g_turnYawStarted != 0U) &&
                 (IMU_IsReady() != 0U)) ?
                ((yawProgress >= LEFT_TURN_LINE_GUARD_YAW_X10) ? 1U : 0U) :
                ((g_turnMs >= LEFT_TURN_LINE_FALLBACK_MS) ? 1U : 0U);

            if ((lineCaptureAllowed != 0U) && (centerSeen != 0U)) {
                if (g_turnLineCaptureConfirm < 255U) {
                    g_turnLineCaptureConfirm++;
                }
            } else {
                g_turnLineCaptureConfirm = 0U;
            }

            if (g_turnLineCaptureConfirm >= LEFT_TURN_LINE_CAPTURE_COUNT) {
                finish_left_turn();
                return;
            }
        }

        if ((g_turnYawStarted == 0U) &&
            (g_turnMs >= LEFT_TURN_OPEN_LOOP_MS)) {
            finish_left_turn();
            return;
        }

        if (g_turnMs >= LEFT_TURN_MAX_MS) {
            finish_left_turn();
            return;
        }

        if ((g_turnYawStarted != 0U) && (IMU_IsReady() != 0U)) {
            if ((yawProgress < LEFT_TURN_FINAL_START_YAW_X10) ||
                (g_turnMs < LEFT_TURN_MIN_MS)) {
                g_leftOut = LEFT_TURN_LEFT_PWM_FAST;
                g_rightOut = LEFT_TURN_RIGHT_PWM_FAST;
            } else {
                g_leftOut = LEFT_TURN_LEFT_PWM_FINAL;
                g_rightOut = LEFT_TURN_RIGHT_PWM_FINAL;
            }
        } else if (g_turnMs < LEFT_TURN_OPEN_LOOP_FAST_MS) {
            g_leftOut = LEFT_TURN_LEFT_PWM_FAST;
            g_rightOut = LEFT_TURN_RIGHT_PWM_FAST;
        } else {
            g_leftOut = LEFT_TURN_LEFT_PWM_FINAL;
            g_rightOut = LEFT_TURN_RIGHT_PWM_FINAL;
        }
        MOTOR_SetPWM(g_leftOut, g_rightOut);

        g_status.leftTargetMmps = (int16_t)-TURN_SPEED_MMPS;
        g_status.rightTargetMmps = TURN_SPEED_MMPS;
        g_status.leftPwm = g_leftOut;
        g_status.rightPwm = g_rightOut;
        g_status.lineError = yawProgress;
        
        return;
    }

    if (g_mode == MODE_LEFT_TURN_EXIT) {
        int16_t exitTurnPwm;
        int16_t exitBasePwm = calc_feedforward_pwm(LEFT_TURN_EXIT_SPEED_MMPS,
                                                   MIN_RUN_PWM_LEFT);

        g_turnMs += 5U;

        if (exitBasePwm < LEFT_TURN_EXIT_MIN_PWM) {
            exitBasePwm = LEFT_TURN_EXIT_MIN_PWM;
        }

        exitTurnPwm = calc_line_turn(sensorMask);
        exitTurnPwm = clamp_i16(exitTurnPwm,
                                (int16_t)-LEFT_TURN_EXIT_LIMIT_PWM,
                                LEFT_TURN_EXIT_LIMIT_PWM);

        if (stable_line_for_corner_arm(sensorMask) != 0U) {
            if (g_exitLineConfirm < 255U) {
                g_exitLineConfirm++;
            }
        } else {
            g_exitLineConfirm = 0U;
        }

        g_leftOut = clamp_i16((int16_t)(exitBasePwm + exitTurnPwm),
                              0,
                              PWM_MAX_OUT);
        g_rightOut = clamp_i16((int16_t)(exitBasePwm - exitTurnPwm),
                               0,
                               PWM_MAX_OUT);
        MOTOR_SetPWM(g_leftOut, g_rightOut);

        g_status.leftTargetMmps = LEFT_TURN_EXIT_SPEED_MMPS;
        g_status.rightTargetMmps = LEFT_TURN_EXIT_SPEED_MMPS;
        g_status.leftPwm = g_leftOut;
        g_status.rightPwm = g_rightOut;
        g_status.lineError = g_lineErrFilt;

        if ((g_turnMs >= LEFT_TURN_EXIT_MS) &&
            (g_exitLineConfirm >= LEFT_TURN_EXIT_LINE_COUNT)) {
            if (finish_left_turn_exit() != 0U) {
                return;
            }
        } else if (g_turnMs >= LEFT_TURN_EXIT_MAX_MS) {
            g_mode = MODE_LINE;
            g_turnMs = 0U;
            g_exitLineConfirm = 0U;
            g_leftCornerArmed = 0U;
            g_leftCornerArmConfirm = 0U;
            g_leftCornerLatchMs = 0U;
            g_lineDriveUnlocked = 1U;
            g_leftSpeedI = 0;
            g_rightSpeedI = 0;
        }
        return;
    }

    int16_t lineTurnPwm = calc_line_turn(sensorMask);

    update_line_diag(leftSpeed, rightSpeed, lineTurnPwm, sensorMask);

    if (update_left_corner_detector(sensorMask) != 0U) {
        enter_left_turn();
        return;
    }

    if ((g_lapCompletePending != 0U) &&
        (g_leftCornerArmed != 0U) &&
        (complete_pending_lap() != 0U)) {
        return;
    }
    
    if (g_lineLostMs > LINE_LOST_HOLD_MS) {
        g_leftSpeedI = 0;
        g_rightSpeedI = 0;
    } else if ((g_lineDriveUnlocked == 0U) &&
               (g_lineSeenConfirm >= LINE_START_CONFIRM_COUNT)) {
        g_lineDriveUnlocked = 1U;
    }
    
    g_leftSpeedFilt = leftSpeed;
    g_rightSpeedFilt = rightSpeed;
    
    int16_t baseSpeed = 0;

    if (g_lineDriveUnlocked != 0U) {
        baseSpeed = (g_lineLostMs > LINE_LOST_HOLD_MS) ?
                    LINE_RECOVERY_SPEED_MMPS :
                    TARGET_SPEED_MMPS;
    }

    int16_t targetSpeedLeft;
    int16_t targetSpeedRight;
    int16_t turnPwm;

    if (baseSpeed <= 0) {
        targetSpeedLeft = 0;
        targetSpeedRight = 0;
        turnPwm = 0;
    } else {
        int16_t maxTurnSpeed = (int16_t)(baseSpeed -
                                         LINE_MIN_INNER_SPEED_MMPS);
        int16_t wheelDiffSpeed = (int16_t)(g_leftSpeedFilt -
                                           g_rightSpeedFilt);
        int16_t wheelDiffDamp =
            (int16_t)(((int32_t)LINE_WHEEL_DIFF_DAMP_NUM *
                       wheelDiffSpeed) /
                      LINE_WHEEL_DIFF_DAMP_DIV);
        int16_t turnSpeed;

        turnPwm = (int16_t)clamp_i32((int32_t)lineTurnPwm,
                                     -LINE_TURN_LIMIT,
                                     LINE_TURN_LIMIT);

        if (maxTurnSpeed < 0) {
            maxTurnSpeed = 0;
        }

        turnSpeed = (int16_t)(((int32_t)turnPwm *
                               LINE_TURN_SPEED_NUM) /
                              LINE_TURN_SPEED_DIV);
        turnSpeed = (int16_t)(turnSpeed - wheelDiffDamp);
        turnSpeed = clamp_i16(turnSpeed,
                              (int16_t)-maxTurnSpeed,
                              maxTurnSpeed);

        targetSpeedLeft = (int16_t)(baseSpeed + turnSpeed);
        targetSpeedRight = (int16_t)(baseSpeed - turnSpeed);
    }

    if (STRAIGHT_USE_ENCODER_SPEED_LOOP != 0) {
        g_leftOut = calc_speed_pwm(targetSpeedLeft,
                                   g_leftSpeedFilt,
                                   &g_leftSpeedI,
                                   MIN_RUN_PWM_LEFT);
        g_rightOut = calc_speed_pwm(targetSpeedRight,
                                    g_rightSpeedFilt,
                                    &g_rightSpeedI,
                                    MIN_RUN_PWM_RIGHT);
    } else {
        g_leftOut = calc_feedforward_pwm(targetSpeedLeft, MIN_RUN_PWM_LEFT);
        g_rightOut = calc_feedforward_pwm(targetSpeedRight, MIN_RUN_PWM_RIGHT);
    }

    g_leftOut = clamp_i16((int16_t)(g_leftOut - STRAIGHT_TRIM_PWM),
                          (int16_t)-SPEED_BRAKE_LIMIT_PWM,
                          PWM_MAX_OUT);
    g_rightOut = clamp_i16((int16_t)(g_rightOut + STRAIGHT_TRIM_PWM),
                           (int16_t)-SPEED_BRAKE_LIMIT_PWM,
                           PWM_MAX_OUT);

    MOTOR_SetPWM(g_leftOut, g_rightOut);

    g_status.leftTargetMmps = targetSpeedLeft;
    g_status.rightTargetMmps = targetSpeedRight;
    g_status.leftPwm = g_leftOut;
    g_status.rightPwm = g_rightOut;
    g_status.lineError = g_lineErrFilt;
}

void RACE_GetStatus(RACE_Status_t *status)
{
    if (status == 0) return;
    uint32_t primask = irq_save();
    *status = g_status;
    irq_restore(primask);
}

void RACE_SetTargetLaps(uint8_t laps)
{
    if (laps < 1U) laps = 1U;
    if (laps > 5U) laps = 5U;
    g_status.targetLaps = laps;
}

void RACE_NextTargetLap(void)
{
    uint8_t laps = g_status.targetLaps;
    if (laps < 1U) laps = 1U;
    laps++;
    if (laps > 5U) laps = 1U;
    g_status.targetLaps = laps;
}
