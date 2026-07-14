/*
 * 文件：lab_ctrl.c
 * 用途：上位机 PID 调参实验的核心控制模块。
 *
 * 本文件统一管理串口命令、左右轮 PI 闭环、直线同步、八路灰度循迹、
 * 逆时针方框状态机、板载按键脱机启动、数据流/离线日志以及参数 Flash 保存。
 * 时序：LAB_Task() 在 main 主循环中高频调用；真正的速度闭环只在编码器产生
 * 新的 10 ms 速度样本时更新，避免对同一帧测速结果重复积分。
 * 安全：所有模式切换先停电机并清 PI 积分；开环 PWM 有超时保护；方框模式同时
 * 使用 IMU、编码器里程和重新捕线判断转角，任一关键条件异常会安全停车。
 */
#include "lab_ctrl.h"
#include "app_config.h"
#include "serial.h"
#include "motor.h"
#include "encoder.h"
#include "sensor.h"
#include "imu.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>

/* ===============================================================
 * 常量定义
 * 大部分限幅、状态时间都在这里集中维护
 * =============================================================== */

/* 命令行解析时单条命令允许的最大 token 数 */
#define LAB_LINE_SIZE               64U

/* 开环 PWM 与电机输出模块使用相同上限。PWM 周期为 1000，300 只有 30%
 * 占空比，无法保证空载电机起转，因此不再人为截到 300。 */
#define LAB_PWM_OPEN_LIMIT          620
/* 闭环 PI 输出占空比上限（绝对值） */
#define LAB_PWM_CLOSED_LIMIT        500

/* 开环模式最大持续时间，超时后强制停转（安全保护） */
#define LAB_OPEN_TIMEOUT_MS        2000U

/* 数据流式发送周期 / 离线日志采样周期 */
#define LAB_STREAM_PERIOD_MS         20U
/* 自动轮速整定在主控本地采样；无线端只接收最终汇总，避免 ESP-NOW
 * 对实时 CSV 的排队延迟影响参数选择。 */
#define LAB_TUNE_SAMPLE_PERIOD_MS    20U

/* PI 控制器外环周期（与 ENCODER 测速一致），用于把误差换算成“误差毫秒” */
#define LAB_CONTROL_PERIOD_MS        10

/* PC 端只接受相同版本的可靠事务协议，避免新旧固件混用后静默失控。 */
#define LAB_PROTOCOL_VERSION          2

/* 悬空轮速辨识的硬件保护阈值。正常工作区约 80~600 mm/s；达到此值说明
 * 扫描已经远离有效工作区或编码器标定异常，固件会立即结束该测试点。 */
#define LAB_TUNE_OPEN_STOP_MMPS    1800
#define LAB_TUNE_STEP_STOP_MMPS     900

#define LAB_TUNE_FLAG_OVERSPEED    0x01U
#define LAB_TUNE_FLAG_CLIPPED      0x02U
#define LAB_TOKEN_MAX       2147483647L

/* PI 积分累加的限幅，避免积分饱和 */
#define LAB_INTEGRAL_LIMIT_MS    500000L

/* 直线同步外环的速度修正上限（mm/s） */
#define LAB_SYNC_LIMIT_MMPS          60

/* 上位机可以下发的最小/最大目标速度（mm/s） */
#define LAB_SPEED_MIN_MMPS           80
#define LAB_SPEED_MAX_MMPS          600

/* 方框模式下的最大目标速度（mm/s） */
#define LAB_SQUARE_MAX_SPEED_MMPS   450

/* 闭环测试时长的合法范围 */
#define LAB_TEST_MIN_MS             500U
#define LAB_TEST_MAX_MS           15000U

/* 离线日志环形缓冲区最多能保存的样本数 */
#define LAB_LOG_CAPACITY            600U

/* 离线日志 DUMP 时每条样本之间的时间间隔（避免刷屏太快） */
#define LAB_DUMP_PERIOD_MS            8U

/* 按键消抖时间 */
#define LAB_KEY_DEBOUNCE_MS          25U
/* ARM 或脱机方框按 SW3 后的安全启动延迟 */
#define LAB_ARM_START_DELAY_MS      2000U

/* 闭环启动时的速度斜坡时间，避免瞬时阶跃 */
#define LAB_START_RAMP_MS           1500U

/* 循迹模式下完全失去线的最长时间：超时后停转 */
#define LAB_LINE_LOST_STOP_MS        600U
/* 循迹模式失去线后进入“降速巡航”的时间 */
#define LAB_LINE_RECOVERY_MS         100U
/* 降速巡航时的速度上限 */
#define LAB_LINE_RECOVERY_MMPS        100

/* 循迹外环输出速度修正的限幅 */
#define LAB_LINE_TURN_LIMIT_MMPS      160
/* 高速循迹阈值：基础速度超过该值时切换到高速调参 */
#define LAB_LINE_HIGH_SPEED_MMPS      420
/* 高速循迹下更紧的速度修正限幅 */
#define LAB_LINE_HIGH_TURN_LIMIT_MMPS  90
/* 高速循迹下目标修正量的爬升速率限制（mm/s / 步） */
#define LAB_LINE_HIGH_TURN_SLEW_MMPS   15

/* 灰度有效位掩码：低 7 位为有效检测位 */
#define LAB_LINE_SENSOR_VALID_MASK   0x7FU

/* 误差绝对值在以下区间时被认为“非常接近线” */
#define LAB_LINE_CENTER_ERROR            4
/* 误差绝对值超过该值时被认为“严重偏离线” */
#define LAB_LINE_FAR_ERROR              10

/* 转弯前“靠近线”的短时推进时间（ms） */
#define LAB_TURN_APPROACH_MS            50U
/* 转弯最短耗时，防止过快地被判定为完成 */
#define LAB_TURN_MIN_MS                220U
/* 转弯最长耗时（总保护） */
#define LAB_TURN_MAX_MS               1400U
/* 转弯完成 -> 重新走直线时，验证线出现的最短时间 */
#define LAB_TURN_EXIT_MIN_MS           120U
/* 验证线的最长时间，超时后报 LINE NOT CAPTURED */
#define LAB_TURN_EXIT_MAX_MS           800U

/* 在进入“转角”判定前，要求连续看到几次“线居中” */
#define LAB_CORNER_ARM_CONFIRM           8U
/* 在“待转角”状态下，要求连续看到几次“左边大块黑” */
#define LAB_CORNER_DETECT_CONFIRM        2U
/* EXIT 阶段验证到中心线的次数 */
#define LAB_TURN_CENTER_CONFIRM          3U
/* 转弯中线被认定为“已离开”的次数 */
#define LAB_TURN_LINE_CLEAR_CONFIRM      2U
/* 转弯中线被认定为“已重新看到”的次数 */
#define LAB_TURN_LINE_CAPTURE_CONFIRM    3U

/* 单次采样 yaw 的最大允许跳变（0.1°），用于判定 IMU 是否出现野点 */
#define LAB_TURN_YAW_MAX_STEP_X10         80

/* 转弯中“按里程提前结束”的里程裕量（mm） */
#define LAB_TURN_DISTANCE_SLOW_MARGIN_MM  25

/* 边长的初始猜测 / 最小值 / 最大值（mm），用于在线估算边长 */
#define LAB_EDGE_MIN_INITIAL_MM           450
#define LAB_EDGE_MIN_LIMIT_MM             300
#define LAB_EDGE_MAX_LIMIT_MM             900

/* 脱机（不接 USB）独立跑方框时的速度 */
#define LAB_STANDALONE_SQUARE_SPEED       300
/* 上位机可指定的最大方框圈数 */
#define LAB_SQUARE_MAX_LAPS             10U
#define LAB_STANDALONE_MAX_LAPS LAB_SQUARE_MAX_LAPS

/* Flash 烧写区地址（最后一页附近） */
#define LAB_FLASH_ADDRESS       0x0001FC00U
/* 参数魔数：用以识别“参数区是否已写过” */
#define LAB_FLASH_MAGIC         0x50494431U
/* 参数区结构版本号，便于将来扩展时识别 */
#define LAB_FLASH_VERSION_V1             1U
#define LAB_FLASH_VERSION_V2             2U
#define LAB_FLASH_VERSION                LAB_FLASH_VERSION_V2

/* ===============================================================
 * 枚举与结构体
 * =============================================================== */

/* 当前上位机控制的运行模式 */
typedef enum {
    LAB_MODE_IDLE = 0,        /* 空闲：电机停转 */
    LAB_MODE_OPEN_PWM,        /* 开环 PWM 测试 */
    LAB_MODE_STEP_LEFT,       /* 阶跃：左轮转 */
    LAB_MODE_STEP_RIGHT,      /* 阶跃：右轮转 */
    LAB_MODE_STRAIGHT,        /* 直线行驶（带同步外环） */
    LAB_MODE_LINE,            /* 循迹 */
    LAB_MODE_SQUARE           /* 逆时针方框 */
} LabMode_t;

/* 方框模式子状态机 */
typedef enum {
    SQUARE_STATE_LINE = 0,        /* 沿直线行驶 */
    SQUARE_STATE_APPROACH,        /* 进入转角前的短直行 */
    SQUARE_STATE_TURN_FAST,       /* 快速转向（大半径） */
    SQUARE_STATE_TURN_SLOW,       /* 接近完成时降速转向 */
    SQUARE_STATE_EXIT             /* 转向后回到直线的过渡 */
} SquareState_t;

/* 单个车轮的 PI 控制器状态 */
typedef struct {
    int32_t kpX1000;        /* 比例增益 Kp（x1000 缩放） */
    int32_t kiX1000;        /* 积分增益 Ki（x1000 缩放） */
    int32_t ffX1000;        /* 前馈增益 FF（x1000 缩放） */
    int16_t minPwm;         /* 最小启动 PWM */
    int32_t integralErrMs;  /* 积分累加值（误差 * 周期） */
} WheelPi_t;

/* 烧写到 Flash 的全部可调参数 */
typedef struct {
    uint32_t magic;
    uint32_t version;
    int32_t leftKpX1000;
    int32_t leftKiX1000;
    int32_t rightKpX1000;
    int32_t rightKiX1000;
    int32_t leftFfX1000;
    int32_t rightFfX1000;
    int32_t leftMinPwm;
    int32_t rightMinPwm;
    int32_t syncKpX1000;
    int32_t rightBiasMmps;
    uint32_t checksum;
    uint32_t reserved;          /* 复用：低 16 位是 LINEKP，高 16 位是 LINEKD */
    int32_t turnAngleX10;
    int32_t turnFastPwm;
    int32_t turnSlowPwm;
    int32_t turnSlowMarginX10;
    int32_t turnExitMmps;
    int32_t turnDistanceMm;
} LabFlashConfigV1_t;

/* V2 不再复用字段：所有控制参数均有独立成员并参与完整校验。
 * structSize 用于防止未来结构扩展时把不同布局的数据误当成当前版本。 */
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t structSize;
    int32_t leftKpX1000;
    int32_t leftKiX1000;
    int32_t rightKpX1000;
    int32_t rightKiX1000;
    int32_t leftFfX1000;
    int32_t rightFfX1000;
    int32_t leftMinPwm;
    int32_t rightMinPwm;
    int32_t syncKpX1000;
    int32_t rightBiasMmps;
    int32_t lineKpX1000;
    int32_t lineKdX1000;
    int32_t turnAngleX10;
    int32_t turnFastPwm;
    int32_t turnSlowPwm;
    int32_t turnSlowMarginX10;
    int32_t turnExitMmps;
    int32_t turnDistanceMm;
    uint32_t checksum;
} LabFlashConfigV2_t;

/* 单条离线日志样本 */
typedef struct {
    uint16_t timeMs;
    int16_t leftTarget;
    int16_t rightTarget;
    int16_t leftSpeed;
    int16_t rightSpeed;
    int16_t leftPwm;
    int16_t rightPwm;
    int16_t leftError;
    int16_t rightError;
    int16_t countDiff;
} LabLogSample_t;

typedef enum {
    LAB_TUNE_NONE = 0,
    LAB_TUNE_OPEN,
    LAB_TUNE_STEP
} LabTuneKind_t;

typedef struct {
    LabTuneKind_t kind;
    char wheel;
    int16_t commandValue;
    int16_t targetSpeed;
    uint32_t startMs;
    uint32_t durationMs;
    uint32_t nextSampleMs;
    uint16_t sampleCount;
    uint16_t tailCount;
    uint16_t saturationCount;
    uint32_t token;
    uint32_t riseTimeMs;
    int16_t maxSpeed;
    int64_t sumSpeed;
    int64_t sumAbsError;
    int64_t tailSum;
    int64_t tailSquareSum;
    uint8_t flags;
} LabTuneMetrics_t;

/* ===============================================================
 * 静态变量
 * =============================================================== */

/* 当前模式 */
static LabMode_t g_mode = LAB_MODE_IDLE;

/* 左右轮各自的 PI 状态 */
static WheelPi_t g_leftPi;
static WheelPi_t g_rightPi;

/* 最近一次下发到电机的 PWM */
static int16_t g_leftPwm = 0;
static int16_t g_rightPwm = 0;
/* 左右轮目标速度（mm/s），由控制算法计算 */
static int16_t g_leftTarget = 0;
static int16_t g_rightTarget = 0;
/* 阶跃 / 直线 / 循迹 / 方框 测试时的“基础目标速度” */
static int16_t g_testSpeed = 0;

/* 直线同步外环的机械偏置（mm/s 等价） */
static int16_t g_rightBiasMmps = LAB_DEFAULT_RIGHT_BIAS_MMPS;
/* 同步外环 Kp */
static int32_t g_syncKpX1000 = LAB_DEFAULT_SYNC_KP_X1000;

/* 循迹外环的 Kp / Kd */
static int32_t g_lineKpX1000 = LAB_DEFAULT_LINE_KP_X1000;
static int32_t g_lineKdX1000 = LAB_DEFAULT_LINE_KD_X1000;

/* 当前 / 上一次的循迹误差（加权后的中心偏差） */
static int16_t g_lineError = 0;
static int16_t g_linePreviousError = 0;
/* 循迹外环输出的左右速度修正（mm/s） */
static int16_t g_lineTurnMmps = 0;
/* 失去线的累计时间（ms） */
static uint16_t g_lineLostMs = 0U;
/* 最近一次循迹误差是否有效 */
static uint8_t g_lineValid = 0U;
/* 最近一次的灰度掩码 */
static uint8_t g_lineMask = 0U;

/* 上位机 KICK 命令：短时间内强制附加一个固定的左右差速 */
static int16_t g_lineKickMmps = 0;
static uint32_t g_lineKickEndMs = 0U;

/* 方框模式下的转弯相关参数 */
static int16_t g_turnAngleX10 = LAB_DEFAULT_TURN_ANGLE_X10;
static int16_t g_turnFastPwm = LAB_DEFAULT_TURN_FAST_PWM;
static int16_t g_turnSlowPwm = LAB_DEFAULT_TURN_SLOW_PWM;
static int16_t g_turnSlowMarginX10 =
    LAB_DEFAULT_TURN_SLOW_MARGIN_X10;
static int16_t g_turnExitMmps = LAB_DEFAULT_TURN_EXIT_MMPS;
static int16_t g_turnDistanceMm = LAB_DEFAULT_TURN_DISTANCE_MM;

/* 方框状态机相关 */
static SquareState_t g_squareState = SQUARE_STATE_LINE;
static uint32_t g_squareStateStartMs = 0U;
static uint32_t g_squareTurnStartMs = 0U;
static uint8_t g_squareTargetLaps = 1U;        /* 目标圈数 */
static uint8_t g_squareCornerCount = 0U;       /* 已完成转角数 */
/* 转弯前的“线居中”/“左大块黑”连续确认计数 */
static uint8_t g_cornerArmCount = 0U;
static uint8_t g_cornerDetectCount = 0U;
static uint8_t g_turnCenterCount = 0U;
/* 当前 / 上一次接受的相对 yaw（0.1°） */
static int16_t g_turnYawX10 = 0;
static int16_t g_turnLastAcceptedYawX10 = 0;
static uint8_t g_turnYawReliable = 0U;         /* 当前 IMU 读数是否可信 */
static uint8_t g_turnLineCleared = 0U;         /* 转弯中线是否已经离开 */
static uint8_t g_turnLineClearCount = 0U;
static uint8_t g_turnLineCaptureCount = 0U;
/* 转弯开始时编码器里程的快照 */
static int32_t g_turnStartLeftMm = 0;
static int32_t g_turnStartRightMm = 0;
/* 当前转弯已经走过的平均里程 */
static int16_t g_turnTravelMm = 0;
/* 当前转弯的结束条件是否由“线重新出现”触发 */
static uint8_t g_turnCapturedByLine = 0U;

/* 当前直行段的起止里程（用于估算边长） */
static int32_t g_edgeStartLeftMm = 0;
static int32_t g_edgeStartRightMm = 0;
static int16_t g_edgeTravelMm = 0;
/* 在线学习到的“最短边长”阈值 */
static int16_t g_cornerMinEdgeMm = LAB_EDGE_MIN_INITIAL_MM;

/* ===============================================================
 * 实时日志 / 数据流相关
 * =============================================================== */

static uint8_t g_streamEnabled = 0U;           /* 是否周期发送 PID 流 */
static uint8_t g_compactTuneStream = 0U;       /* 循迹整定专用短遥测 */
static uint32_t g_modeStartMs = 0U;             /* 当前模式开始时间 */
static uint32_t g_modeDurationMs = 0U;          /* 当前模式计划持续时间 */
static uint32_t g_lastMotorCommandMs = 0U;      /* 开环 PWM 最近一次命令时间（用于超时） */
static uint32_t g_lastStreamMs = 0U;            /* 上次流式发送时间 */
static uint32_t g_lastSpeedSequence = 0U;       /* 上次处理过的编码器速度序列号 */
static uint32_t g_lastLogMs = 0U;               /* 上次写入离线日志时间 */
static uint32_t g_lastDumpMs = 0U;              /* 上次 DUMP 发送时间 */

static LabLogSample_t g_runLog[LAB_LOG_CAPACITY];
static uint16_t g_runLogCount = 0U;             /* 当前已写入条数 */
static uint16_t g_dumpIndex = 0U;               /* DUMP 当前发送到的下标 */
static uint8_t g_captureLog = 0U;               /* 是否正在记录离线日志 */
static uint8_t g_dumpActive = 0U;               /* 是否正在回放日志 */
static LabTuneMetrics_t g_tuneMetrics;
static LabTuneMetrics_t g_lastTuneResult;
static uint8_t g_lastTuneResultValid = 0U;
/* SAVE token 的结果缓存使 ACK 丢失后的重发不会再次擦写 Flash。 */
static uint32_t g_lastSaveToken = 0U;
static uint8_t g_lastSaveTokenValid = 0U;
static uint8_t g_lastSaveSucceeded = 0U;
/* SQUARE 启动也做幂等缓存，ACK 丢失后的相同 token 不会让小车重新起跑。 */
static uint32_t g_lastSquareToken = 0U;
static uint8_t g_lastSquareTokenValid = 0U;
static uint8_t g_lastSquareAccepted = 0U;
static uint32_t g_lastKickToken = 0U;
static uint8_t g_lastKickTokenValid = 0U;

/* ARM 状态：等待 SW3 启动一次离线测试 */
static uint8_t g_straightArmed = 0U;
static uint8_t g_startPending = 0U;
/* 脱机（不接 USB）独立跑方框的待启动标志 */
static uint8_t g_squareStartPending = 0U;
static int16_t g_armedSpeed = 0;
static uint32_t g_armedDurationMs = 0U;
static uint32_t g_pendingStartMs = 0U;

/* 三个独立按键的消抖状态：SW1 减圈、SW2 加圈、SW3 启停。 */
typedef struct {
    uint8_t rawLast;
    uint8_t stable;
    uint8_t lastStable;
    uint16_t sameMs;
} LabKeyState_t;

static LabKeyState_t g_keys[3];
static uint32_t g_lastKeySampleMs = 0U;
static uint8_t g_selectedSquareLaps = 1U;

static uint8_t is_space(char c)
{
    /* 命令解析中允许的空白字符：空格和制表符 */
    return ((c == ' ') || (c == '\t')) ? 1U : 0U;
}

static char ascii_upper(char c)
{
    /* 把小写字母变成大写字母；其他字符保持不变。
     * 这样做可以让命令解析忽略大小写 */
    if ((c >= 'a') && (c <= 'z')) {
        return (char)(c - ('a' - 'A'));
    }
    return c;
}

static void normalize_line(char *line)
{
    /* 把整行命令原地转成大写，并跳过字符串结束符 */
    while (*line != '\0') {
        *line = ascii_upper(*line);
        line++;
    }
}

static uint8_t text_equal(const char *left, const char *right)
{
    /* 不调用 strcmp 的字符串相等比较 */
    while ((*left != '\0') && (*right != '\0')) {
        if (*left != *right) {
            return 0U;
        }
        left++;
        right++;
    }
    return ((*left == '\0') && (*right == '\0')) ? 1U : 0U;
}

static uint8_t split_tokens(char *line, char **tokens, uint8_t capacity)
{
    /* 把 line 拆成最多 capacity 个 token，遇到空白就切分。
     * 注意：该函数会原地修改 line（在 token 末尾写入 \0） */
    uint8_t count = 0U;

    while ((*line != '\0') && (count < capacity)) {
        /* 跳过开头空白 */
        while (is_space(*line) != 0U) {
            line++;
        }
        if (*line == '\0') {
            break;
        }

        /* 记录 token 起点 */
        tokens[count] = line;
        count++;

        /* 找到 token 末尾的空白，替换为 \0 */
        while ((*line != '\0') && (is_space(*line) == 0U)) {
            line++;
        }
        if (*line != '\0') {
            *line = '\0';
            line++;
        }
    }

    return count;
}

static uint8_t parse_i32(const char *text, int32_t *value)
{
    /* 手写的十进制整数解析：
     *   - 可选 '+' / '-' 号
     *   - 解析过程中超过 2e9 直接判失败
     *   - 解析完整字符串（不允许尾部多余字符） */
    int32_t parsed = 0;
    int32_t sign = 1;
    uint8_t hasDigit = 0U;

    if (*text == '-') {
        sign = -1;
        text++;
    } else if (*text == '+') {
        text++;
    }

    while ((*text >= '0') && (*text <= '9')) {
        hasDigit = 1U;
        if (parsed > 200000000L) {
            return 0U;
        }
        parsed = (parsed * 10) + (int32_t)(*text - '0');
        text++;
    }

    if ((hasDigit == 0U) || (*text != '\0')) {
        return 0U;
    }

    *value = parsed * sign;
    return 1U;
}

static int16_t clamp_i16(int32_t value, int16_t minValue, int16_t maxValue)
{
    /* 把 int32 数值钳到 int16 范围。统一用 int32 参与比较避免溢出 */
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return (int16_t)value;
}

static int32_t clamp_i32(int32_t value, int32_t minValue, int32_t maxValue)
{
    /* int32 版本的钳制 */
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

static int16_t abs_i16(int16_t value)
{
    /* int16 取绝对值（不调用 abs 库函数以避免 -INT16_MIN 溢出） */
    return (value >= 0) ? value : (int16_t)(-value);
}

static int16_t square_edge_travel_mm(void)
{
    /* 计算“当前直行段”已经走过的平均里程（mm）：
     *   取左右轮里程的绝对值，再求平均；这样倒车也不会出现负数 */
    int32_t left =
        ENCODER_GetLeftDistanceMm() - g_edgeStartLeftMm;
    int32_t right =
        ENCODER_GetRightDistanceMm() - g_edgeStartRightMm;
    int32_t average;

    if (left < 0) left = -left;
    if (right < 0) right = -right;
    average = (left + right) / 2L;
    return clamp_i16(average, 0, 32767);
}

static uint8_t sensor_bit_active(uint8_t mask, uint8_t index)
{
    /* 取出 8 位掩码中的第 index 位（0/1） */
    return (uint8_t)((mask >> index) & 0x01U);
}

/* 将灰度位掩码换算为加权横向误差（左负、右正）。
 * 多个相邻探头同时压线时取平均；全丢线、过多探头同时触发或左右两侧都有线
 * 而中心无线时，视为路口/异常图形并把 valid 置零，防止 PD 使用错误数据。 */
static int16_t calculate_line_error(uint8_t mask, uint8_t *valid)
{
    /* 把 8 路灰度传感器的位掩码换算成“线居中误差”：
     *   - 每位有一个固定权重，1 表示黑线压在该路
     *   - 求和后除以触发路数得到一个平均位置
     *   - 权重的中心被设计为 0，负数表示线偏左，正数表示线偏右
     *
     * valid 标记本帧是否可信：
     *   - 没有任何触发（丢线）
     *   - 触发数 >= 5（说明线很宽或噪声太大）
     *   - 中心无触发但左右外侧都触发（线整体落在传感器外）
     *   都会被视为无效
     */
    static const int16_t weights[8] = {-36, -25, -11, -2,
                                       2,  11,  36,  0};
    int32_t sum = 0;
    uint8_t count = 0U;
    uint8_t leftOuter = 0U;
    uint8_t rightOuter = 0U;
    uint8_t center = 0U;
    uint8_t i;

    *valid = 0U;
    if ((mask == 0U) || (SENSOR_GetI2cOk() == 0U)) {
        return 0;
    }

    for (i = 0U; i < 8U; i++) {
        if (sensor_bit_active(mask, i) != 0U) {
            sum += weights[i];
            count++;
            /* 0/1/2 号位表示“偏左外侧” */
            if (i <= 2U) leftOuter = 1U;
            /* 5/6/7 号位表示“偏右外侧” */
            if (i >= 5U) rightOuter = 1U;
            /* 3/4 号位是中心位 */
            if ((i == 3U) || (i == 4U)) center = 1U;
        }
    }

    if ((count == 0U) || (count >= 5U) ||
        ((center == 0U) && (leftOuter != 0U) &&
         (rightOuter != 0U))) {
        return 0;
    }

    *valid = 1U;
    return (int16_t)(sum / count);
}

static uint8_t stable_center_line(uint8_t mask)
{
    /* 判断“线居中且无极端外侧触发”：
     *   用于方框模式进入转角前的“线很稳”确认 */
    uint8_t valid;
    int16_t error = calculate_line_error(mask, &valid);

    if ((valid == 0U) || (abs_i16(error) > 11) ||
        (sensor_bit_active(mask, 0U) != 0U) ||
        (sensor_bit_active(mask, 1U) != 0U) ||
        (sensor_bit_active(mask, 6U) != 0U)) {
        return 0U;
    }
    return 1U;
}

static uint8_t center_line_seen(uint8_t mask)
{
    /* 只看中心两路是否触发：用于判断“线经过传感器中心” */
    return ((sensor_bit_active(mask, 3U) != 0U) ||
            (sensor_bit_active(mask, 4U) != 0U)) ? 1U : 0U;
}

static uint8_t left_corner_present(uint8_t mask)
{
    /* 判断是否出现“左转角”：
     *   - 左侧 4 路（0~3）至少 3 路触发
     *   - 右侧最外侧 6 号位未触发，避免与“完全在右”混淆
     * 逆时针跑方框时，左侧转角就是我们需要检测的“前方是直角的拐” */
    uint8_t leftCount =
        (uint8_t)(sensor_bit_active(mask, 0U) +
                  sensor_bit_active(mask, 1U) +
                  sensor_bit_active(mask, 2U) +
                  sensor_bit_active(mask, 3U));

    return ((leftCount >= 3U) &&
            (sensor_bit_active(mask, 6U) == 0U)) ? 1U : 0U;
}

static uint32_t flash_checksum_v1(const LabFlashConfigV1_t *config)
{
    /* Flash 参数区的校验和：
     *   用一个固定常数 0x6D2B79F5 与所有关键字段异或。
     *   任何一个字节出错都会让校验和不匹配，从而回退默认值 */
    return 0x6D2B79F5U ^
           config->magic ^
           config->version ^
           (uint32_t)config->leftKpX1000 ^
           (uint32_t)config->leftKiX1000 ^
           (uint32_t)config->rightKpX1000 ^
           (uint32_t)config->rightKiX1000 ^
           (uint32_t)config->leftFfX1000 ^
           (uint32_t)config->rightFfX1000 ^
           (uint32_t)config->leftMinPwm ^
           (uint32_t)config->rightMinPwm ^
           (uint32_t)config->syncKpX1000 ^
           (uint32_t)config->rightBiasMmps;
}

static uint8_t flash_values_valid_v1(const LabFlashConfigV1_t *config)
{
    /* 校验 Flash 中的参数是否在合法范围内（防止擦除后读到全 0xFF） */
    if ((config->leftKpX1000 > 3000) ||
        (config->leftKiX1000 > 3000) ||
        (config->rightKpX1000 > 3000) ||
        (config->rightKiX1000 > 3000) ||
        (config->leftFfX1000 > 1000) ||
        (config->rightFfX1000 > 1000) ||
        (config->leftMinPwm > 450) ||
        (config->rightMinPwm > 450) ||
        (config->syncKpX1000 > 5000) ||
        (config->rightBiasMmps < -80) ||
        (config->rightBiasMmps > 80)) {
        return 0U;
    }

    if ((config->leftKpX1000 < 0) ||
        (config->leftKiX1000 < 0) ||
        (config->rightKpX1000 < 0) ||
        (config->rightKiX1000 < 0) ||
        (config->leftFfX1000 < 0) ||
        (config->rightFfX1000 < 0) ||
        (config->leftMinPwm < 0) ||
        (config->rightMinPwm < 0) ||
        (config->syncKpX1000 < 0)) {
        return 0U;
    }

    return 1U;
}

static uint32_t flash_checksum_v2(const LabFlashConfigV2_t *config)
{
    const uint32_t *words = (const uint32_t *)config;
    const uint32_t count =
        (uint32_t)(sizeof(LabFlashConfigV2_t) / sizeof(uint32_t));
    uint32_t hash = 2166136261U;
    uint32_t index;

    /* FNV-1a 混合全部字段；最后一个 checksum 字段按 0 参与计算。 */
    for (index = 0U; index < count; index++) {
        hash ^= (index == (count - 1U)) ? 0U : words[index];
        hash *= 16777619U;
    }
    return hash;
}

static uint8_t flash_values_valid_v2(const LabFlashConfigV2_t *config)
{
    LabFlashConfigV1_t common;

    /* 复用经过验证的电机参数范围检查。 */
    common.leftKpX1000 = config->leftKpX1000;
    common.leftKiX1000 = config->leftKiX1000;
    common.rightKpX1000 = config->rightKpX1000;
    common.rightKiX1000 = config->rightKiX1000;
    common.leftFfX1000 = config->leftFfX1000;
    common.rightFfX1000 = config->rightFfX1000;
    common.leftMinPwm = config->leftMinPwm;
    common.rightMinPwm = config->rightMinPwm;
    common.syncKpX1000 = config->syncKpX1000;
    common.rightBiasMmps = config->rightBiasMmps;
    if (flash_values_valid_v1(&common) == 0U) return 0U;

    /* V2 将循迹和正方形转弯参数也纳入强制有效性检查。 */
    return ((config->lineKpX1000 >= 0) && (config->lineKpX1000 <= 20000) &&
            (config->lineKdX1000 >= 0) && (config->lineKdX1000 <= 20000) &&
            (config->turnAngleX10 >= 700) && (config->turnAngleX10 <= 1100) &&
            (config->turnFastPwm >= 100) && (config->turnFastPwm <= 300) &&
            (config->turnSlowPwm >= 80) &&
            (config->turnSlowPwm <= config->turnFastPwm) &&
            (config->turnSlowMarginX10 >= 50) &&
            (config->turnSlowMarginX10 <= 350) &&
            (config->turnExitMmps >= 80) && (config->turnExitMmps <= 250) &&
            (config->turnDistanceMm >= 50) &&
            (config->turnDistanceMm <= 140)) ? 1U : 0U;
}

static void apply_flash_v2(const LabFlashConfigV2_t *config)
{
    g_leftPi.kpX1000 = config->leftKpX1000;
    g_leftPi.kiX1000 = config->leftKiX1000;
    g_rightPi.kpX1000 = config->rightKpX1000;
    g_rightPi.kiX1000 = config->rightKiX1000;
    g_leftPi.ffX1000 = config->leftFfX1000;
    g_rightPi.ffX1000 = config->rightFfX1000;
    g_leftPi.minPwm = (int16_t)config->leftMinPwm;
    g_rightPi.minPwm = (int16_t)config->rightMinPwm;
    g_syncKpX1000 = config->syncKpX1000;
    g_rightBiasMmps = (int16_t)config->rightBiasMmps;
    g_lineKpX1000 = config->lineKpX1000;
    g_lineKdX1000 = config->lineKdX1000;
    g_turnAngleX10 = (int16_t)config->turnAngleX10;
    g_turnFastPwm = (int16_t)config->turnFastPwm;
    g_turnSlowPwm = (int16_t)config->turnSlowPwm;
    g_turnSlowMarginX10 = (int16_t)config->turnSlowMarginX10;
    g_turnExitMmps = (int16_t)config->turnExitMmps;
    g_turnDistanceMm = (int16_t)config->turnDistanceMm;
}

/* 从固定 Flash 页加载调参结果。优先校验 V2 完整结构，同时兼容旧 V1；
 * 校验失败时不修改当前默认参数，调用方会继续使用 app_config.h 的保底值。 */
static uint8_t load_flash_parameters(void)
{
    /* 从 Flash 烧写区读出参数、校验、装载到运行时变量。
     * 任何一步失败都返回 0，由调用方决定是否回退到默认参数。 */
    const volatile uint32_t *header =
        (const volatile uint32_t *)(uintptr_t)LAB_FLASH_ADDRESS;
    const volatile LabFlashConfigV1_t *stored =
        (const volatile LabFlashConfigV1_t *)(uintptr_t)LAB_FLASH_ADDRESS;
    LabFlashConfigV1_t config;

    /* V2 优先：校验结构长度、全字段校验和与所有控制参数。 */
    if ((header[0] == LAB_FLASH_MAGIC) &&
        (header[1] == LAB_FLASH_VERSION_V2)) {
        const volatile LabFlashConfigV2_t *storedV2 =
            (const volatile LabFlashConfigV2_t *)(uintptr_t)LAB_FLASH_ADDRESS;
        LabFlashConfigV2_t configV2 = *storedV2;

        if ((configV2.structSize != sizeof(LabFlashConfigV2_t)) ||
            (configV2.checksum != flash_checksum_v2(&configV2)) ||
            (flash_values_valid_v2(&configV2) == 0U)) {
            return 0U;
        }
        apply_flash_v2(&configV2);
        return 1U;
    }

    /* 逐字段拷贝到 RAM，避免直接对 volatile 结构体做多次访问 */
    config.magic = stored->magic;
    config.version = stored->version;
    config.leftKpX1000 = stored->leftKpX1000;
    config.leftKiX1000 = stored->leftKiX1000;
    config.rightKpX1000 = stored->rightKpX1000;
    config.rightKiX1000 = stored->rightKiX1000;
    config.leftFfX1000 = stored->leftFfX1000;
    config.rightFfX1000 = stored->rightFfX1000;
    config.leftMinPwm = stored->leftMinPwm;
    config.rightMinPwm = stored->rightMinPwm;
    config.syncKpX1000 = stored->syncKpX1000;
    config.rightBiasMmps = stored->rightBiasMmps;
    config.checksum = stored->checksum;
    config.reserved = stored->reserved;
    config.turnAngleX10 = stored->turnAngleX10;
    config.turnFastPwm = stored->turnFastPwm;
    config.turnSlowPwm = stored->turnSlowPwm;
    config.turnSlowMarginX10 = stored->turnSlowMarginX10;
    config.turnExitMmps = stored->turnExitMmps;
    config.turnDistanceMm = stored->turnDistanceMm;

    /* 魔数 / 版本 / 校验和 / 数值范围 任意一项不通过都判定为无效 */
    if ((config.magic != LAB_FLASH_MAGIC) ||
        (config.version != LAB_FLASH_VERSION_V1) ||
        (config.checksum != flash_checksum_v1(&config)) ||
        (flash_values_valid_v1(&config) == 0U)) {
        return 0U;
    }

    /* 把所有字段装到运行时变量 */
    g_leftPi.kpX1000 = config.leftKpX1000;
    g_leftPi.kiX1000 = config.leftKiX1000;
    g_rightPi.kpX1000 = config.rightKpX1000;
    g_rightPi.kiX1000 = config.rightKiX1000;
    g_leftPi.ffX1000 = config.leftFfX1000;
    g_rightPi.ffX1000 = config.rightFfX1000;
    g_leftPi.minPwm = (int16_t)config.leftMinPwm;
    g_rightPi.minPwm = (int16_t)config.rightMinPwm;
    g_syncKpX1000 = config.syncKpX1000;
    g_rightBiasMmps = (int16_t)config.rightBiasMmps;

    /* reserved 字段：低 16 位是 LINEKP，高 16 位是 LINEKD。
     * 擦除态为 0xFFFFFFFF，需要单独判断。 */
    if (config.reserved != 0xFFFFFFFFU) {
        int32_t lineKp = (int32_t)(config.reserved & 0xFFFFU);
        int32_t lineKd = (int32_t)((config.reserved >> 16) & 0xFFFFU);
        if ((lineKp <= 20000) && (lineKd <= 20000)) {
            g_lineKpX1000 = lineKp;
            g_lineKdX1000 = lineKd;
        }
    }

    /* 装载方框模式相关参数（带范围检查） */
    if ((config.turnAngleX10 >= 700) &&
        (config.turnAngleX10 <= 1100) &&
        (config.turnFastPwm >= 100) &&
        (config.turnFastPwm <= 300) &&
        (config.turnSlowPwm >= 80) &&
        (config.turnSlowPwm <= config.turnFastPwm) &&
        (config.turnSlowMarginX10 >= 50) &&
        (config.turnSlowMarginX10 <= 350) &&
        (config.turnExitMmps >= 80) &&
        (config.turnExitMmps <= 250)) {
        g_turnAngleX10 = (int16_t)config.turnAngleX10;
        g_turnFastPwm = (int16_t)config.turnFastPwm;
        g_turnSlowPwm = (int16_t)config.turnSlowPwm;
        g_turnSlowMarginX10 = (int16_t)config.turnSlowMarginX10;
        g_turnExitMmps = (int16_t)config.turnExitMmps;
    }
    if ((config.turnDistanceMm >= 50) &&
        (config.turnDistanceMm <= 140)) {
        g_turnDistanceMm = (int16_t)config.turnDistanceMm;
    }
    return 1U;
}

/* 将当前全部可调控制参数打包为 V2 结构、计算校验和并擦写 Flash。
 * 仅由串口 SAVE 命令触发，避免控制过程中频繁擦写 Flash。 */
static uint8_t save_flash_parameters(void)
{
    /* 把当前所有可调参数打包写入 Flash，烧写前会关闭中断。
     * 烧写完成后再读回一次，确保下一次上电可以加载。 */
    LabFlashConfigV2_t config __attribute__((aligned(8)));
    DL_FLASHCTL_COMMAND_STATUS status;
    uint32_t primask;

    config.magic = LAB_FLASH_MAGIC;
    config.version = LAB_FLASH_VERSION;
    config.structSize = sizeof(LabFlashConfigV2_t);
    config.leftKpX1000 = g_leftPi.kpX1000;
    config.leftKiX1000 = g_leftPi.kiX1000;
    config.rightKpX1000 = g_rightPi.kpX1000;
    config.rightKiX1000 = g_rightPi.kiX1000;
    config.leftFfX1000 = g_leftPi.ffX1000;
    config.rightFfX1000 = g_rightPi.ffX1000;
    config.leftMinPwm = g_leftPi.minPwm;
    config.rightMinPwm = g_rightPi.minPwm;
    config.syncKpX1000 = g_syncKpX1000;
    config.rightBiasMmps = g_rightBiasMmps;
    /* 把 LINEKP/LINEKD 塞到 reserved 字段里，节省一个 4 字节槽位 */
    config.lineKpX1000 = g_lineKpX1000;
    config.lineKdX1000 = g_lineKdX1000;
    config.turnAngleX10 = g_turnAngleX10;
    config.turnFastPwm = g_turnFastPwm;
    config.turnSlowPwm = g_turnSlowPwm;
    config.turnSlowMarginX10 = g_turnSlowMarginX10;
    config.turnExitMmps = g_turnExitMmps;
    config.turnDistanceMm = g_turnDistanceMm;
    /* 必须最后计算，确保正方形与循迹参数全部受校验保护。 */
    config.checksum = flash_checksum_v2(&config);

    /* 烧写 Flash 期间不能被任何中断打断 */
    primask = __get_PRIMASK();
    __disable_irq();

    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(FLASHCTL, LAB_FLASH_ADDRESS,
                                DL_FLASHCTL_REGION_SELECT_MAIN);
    status = DL_FlashCTL_eraseMemoryFromRAM(
        FLASHCTL, LAB_FLASH_ADDRESS, DL_FLASHCTL_COMMAND_SIZE_SECTOR);

    if (status == DL_FLASHCTL_COMMAND_STATUS_PASSED) {
        status =
            DL_FlashCTL_programMemoryBlockingFromRAM64WithECCGenerated(
                FLASHCTL, LAB_FLASH_ADDRESS, (uint32_t *)&config,
                (uint32_t)(sizeof(config) / sizeof(uint32_t)),
                DL_FLASHCTL_REGION_SELECT_MAIN);
    }

    /* 烧写结束，恢复先前的中断状态 */
    if (primask == 0U) {
        __enable_irq();
    }

    if (status != DL_FLASHCTL_COMMAND_STATUS_PASSED) {
        return 0U;
    }

    /* 重新加载一遍：让 Flash 立即成为运行时参数的“真实来源” */
    return load_flash_parameters();
}

static const char *mode_name(void)
{
    /* 把当前模式转换成字符串，方便日志显示 */
    switch (g_mode) {
    case LAB_MODE_OPEN_PWM:
        return "PWM";
    case LAB_MODE_STEP_LEFT:
        return "STEP_L";
    case LAB_MODE_STEP_RIGHT:
        return "STEP_R";
    case LAB_MODE_STRAIGHT:
        return "RUN";
    case LAB_MODE_LINE:
        return "LINE";
    case LAB_MODE_SQUARE:
        return "SQUARE";
    default:
        return "IDLE";
    }
}

static void reset_pi_state(void)
{
    /* 切换模式或修改 PI 参数时调用，清掉积分累加避免跳变 */
    g_leftPi.integralErrMs = 0;
    g_rightPi.integralErrMs = 0;
}

static void stop_motors(void)
{
    /* 立即停机并把与控制相关的状态机全部重置：
     *   - PWM / 目标速度清零
     *   - 切换回 IDLE 模式
     *   - 跑图状态机变量清零
     *   - PI 积分清零
     *   - 调用 MOTOR_Stop() 把实际硬件也关掉 */
    g_leftPwm = 0;
    g_rightPwm = 0;
    g_leftTarget = 0;
    g_rightTarget = 0;
    g_mode = LAB_MODE_IDLE;
    g_modeDurationMs = 0U;
    g_lineKickMmps = 0;
    g_lineKickEndMs = 0U;
    g_squareState = SQUARE_STATE_LINE;
    g_squareStateStartMs = 0U;
    g_squareTurnStartMs = 0U;
    g_cornerArmCount = 0U;
    g_cornerDetectCount = 0U;
    g_turnCenterCount = 0U;
    g_turnYawX10 = 0;
    g_turnLastAcceptedYawX10 = 0;
    g_turnYawReliable = 0U;
    g_turnLineCleared = 0U;
    g_turnLineClearCount = 0U;
    g_turnLineCaptureCount = 0U;
    g_turnTravelMm = 0;
    g_turnCapturedByLine = 0U;
    g_edgeTravelMm = 0;
    reset_pi_state();
    MOTOR_Stop();
}

static int16_t calculate_pi_pwm(WheelPi_t *pi, int16_t target,
                                int16_t measured)
{
    /* 单轮的 PI + 前馈控制：
     *   PWM = minPwm
     *       + FF * target      (前馈)
     *       + Kp * err         (比例)
     *       + Ki * ∫err dt     (积分)
     *
     * 同时带 anti-windup：当输出已经撞到限幅且误差方向继续“推”输出时，
     * 拒绝更新积分项，避免积分饱和。 */

    int32_t error;
    int32_t candidateIntegral;
    int32_t pTerm;
    int32_t iTerm;
    int32_t output;

    if (target <= 0) {
        /* 目标速度非正：当作“停转”，并清积分 */
        pi->integralErrMs = 0;
        return 0;
    }

    error = (int32_t)target - measured;
    /* 把误差按控制周期累加成“误差毫秒”并做钳制 */
    candidateIntegral =
        pi->integralErrMs + (error * LAB_CONTROL_PERIOD_MS);
    candidateIntegral = clamp_i32(candidateIntegral,
                                  -LAB_INTEGRAL_LIMIT_MS,
                                  LAB_INTEGRAL_LIMIT_MS);

    pTerm = (pi->kpX1000 * error) / 1000L;
    iTerm = (pi->kiX1000 * candidateIntegral) / 1000000L;
    output = pi->minPwm +
             ((pi->ffX1000 * target) / 1000L) +
             pTerm + iTerm;

    /* 限幅方向检查：如果继续按当前误差方向会把输出推得更“撞墙”，
     * 就不接受新的积分项（保留旧积分） */
    if (!(((output > LAB_PWM_CLOSED_LIMIT) && (error > 0)) ||
          ((output < 0) && (error < 0)))) {
        pi->integralErrMs = candidateIntegral;
    } else {
        iTerm = (pi->kiX1000 * pi->integralErrMs) / 1000000L;
        output = pi->minPwm +
                 ((pi->ffX1000 * target) / 1000L) +
                 pTerm + iTerm;
    }

    /* 最终再钳一次到闭环上限内 */
    return clamp_i16(output, 0, LAB_PWM_CLOSED_LIMIT);
}

static void send_help(void)
{
    /* 给上位机打印命令帮助信息 */
    SERIAL_SendString("PING | STOP | STATUS | SENSOR | IMU | PARAM | SAVE | LOAD | DUMP\r\n");
    SERIAL_SendString("PWM L/R/B pwm | STEP L/R speed ms | RUN/LINE speed ms | SQUARE speed laps | KICK mmps ms\r\n");
    SERIAL_SendString("ARM speed ms: press SW3 later to start untethered RUN\r\n");
    SERIAL_SendString("SET LKP/LKI/RKP/RKI/LFF/RFF/LMIN/RMIN/SYNC/BIAS/LINEKP/LINEKD/TURNANGLE/TURNFAST/TURNSLOW/TURNMARGIN/TURNEXIT/TURNDIST value\r\n");
}

static void send_parameters(void)
{
    /* 把当前所有可调参数以单行 CSV 形式发回上位机。
     * 字段顺序见 send_pid_header() 里的说明。 */
    SERIAL_SendString("PARAM,x1000,LKP,");
    SERIAL_SendInt32(g_leftPi.kpX1000);
    SERIAL_SendString(",LKI,");
    SERIAL_SendInt32(g_leftPi.kiX1000);
    SERIAL_SendString(",RKP,");
    SERIAL_SendInt32(g_rightPi.kpX1000);
    SERIAL_SendString(",RKI,");
    SERIAL_SendInt32(g_rightPi.kiX1000);
    SERIAL_SendString(",LFF,");
    SERIAL_SendInt32(g_leftPi.ffX1000);
    SERIAL_SendString(",RFF,");
    SERIAL_SendInt32(g_rightPi.ffX1000);
    SERIAL_SendString(",LMIN,");
    SERIAL_SendInt32(g_leftPi.minPwm);
    SERIAL_SendString(",RMIN,");
    SERIAL_SendInt32(g_rightPi.minPwm);
    SERIAL_SendString(",SYNC,");
    SERIAL_SendInt32(g_syncKpX1000);
    SERIAL_SendString(",BIAS,");
    SERIAL_SendInt32(g_rightBiasMmps);
    SERIAL_SendString(",LINEKP,");
    SERIAL_SendInt32(g_lineKpX1000);
    SERIAL_SendString(",LINEKD,");
    SERIAL_SendInt32(g_lineKdX1000);
    SERIAL_SendString(",TURNANGLE,");
    SERIAL_SendInt32(g_turnAngleX10);
    SERIAL_SendString(",TURNFAST,");
    SERIAL_SendInt32(g_turnFastPwm);
    SERIAL_SendString(",TURNSLOW,");
    SERIAL_SendInt32(g_turnSlowPwm);
    SERIAL_SendString(",TURNMARGIN,");
    SERIAL_SendInt32(g_turnSlowMarginX10);
    SERIAL_SendString(",TURNEXIT,");
    SERIAL_SendInt32(g_turnExitMmps);
    SERIAL_SendString(",TURNDIST,");
    SERIAL_SendInt32(g_turnDistanceMm);
    SERIAL_SendString("\r\n");
}

static void send_status(uint32_t nowMs)
{
    /* 单行状态报告：时间戳 / 模式 / 左右目标速度 / 左右实际速度 / 左右 PWM / ARM 状态 */
    SERIAL_SendString("STATUS,");
    SERIAL_SendInt32((int32_t)nowMs);
    SERIAL_SendString(",");
    SERIAL_SendString(mode_name());
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_leftTarget);
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_rightTarget);
    SERIAL_SendString(",");
    SERIAL_SendInt32(ENCODER_GetLeftSpeed());
    SERIAL_SendString(",");
    SERIAL_SendInt32(ENCODER_GetRightSpeed());
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_leftPwm);
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_rightPwm);
    SERIAL_SendString(",");
    SERIAL_SendString((g_straightArmed != 0U) ? "ARMED" : "SAFE");
    SERIAL_SendString(",SELECTED_LAPS,");
    SERIAL_SendInt32(g_selectedSquareLaps);
    SERIAL_SendString(",STBY,");
    SERIAL_SendString((MOTOR_IsDriverEnabled() != 0U) ? "HIGH" : "LOW");
    SERIAL_SendString("\r\n");
}

static void send_sensor_status(void)
{
    /* 把传感器内部状态打包成一行 CSV：I2C 状态 / 有效掩码 / 诊断码 / 地址 / 坏帧数 / 原始字节 */
    uint8_t rawMask = SENSOR_GetRawMask();
    uint8_t effectiveMask = (uint8_t)(rawMask &
                                      LAB_LINE_SENSOR_VALID_MASK);

    SERIAL_SendString("SENSOR,");
    SERIAL_SendString((SENSOR_GetI2cOk() != 0U) ? "OK," : "ERROR,");
    SERIAL_SendInt32(effectiveMask);
    SERIAL_SendString(",");
    SERIAL_SendInt32(SENSOR_GetDiagCode());
    SERIAL_SendString(",");
    SERIAL_SendInt32(SENSOR_GetFoundAddr());
    SERIAL_SendString(",");
    SERIAL_SendInt32(SENSOR_GetBadFrameCount());
    SERIAL_SendString(",");
    SERIAL_SendInt32(rawMask);
    SERIAL_SendString("\r\n");
}

static void send_imu_status(void)
{
    /* 单行 IMU 状态：OK/ERROR、绝对 yaw、成功/失败计数 */
    SERIAL_SendString("IMU,");
    SERIAL_SendString((IMU_IsReady() != 0U) ? "OK," : "ERROR,");
    SERIAL_SendInt32(IMU_GetYawX10());
    SERIAL_SendString(",");
    SERIAL_SendInt32((int32_t)IMU_GetOkCount());
    SERIAL_SendString(",");
    SERIAL_SendInt32((int32_t)IMU_GetErrorCount());
    SERIAL_SendString("\r\n");
}

/* 可靠协议握手。token 由 PC 随机生成，响应中原样返回；同时报告固件真正
 * 使用的编码器量纲，避免把旧固件或错误标定当成可用调参环境。 */
static void send_protocol_hello(uint32_t token)
{
    if (SERIAL_TxCanAccept(100U) == 0U) return;
    SERIAL_SendString("HELLO,");
    SERIAL_SendInt32(token);
    SERIAL_SendString(",");
    SERIAL_SendInt32(LAB_PROTOCOL_VERSION);
    SERIAL_SendString(",STBY,");
    SERIAL_SendString((MOTOR_IsDriverEnabled() != 0U) ? "HIGH" : "LOW");
    SERIAL_SendString(",ENCODER_MM_X1000,");
    SERIAL_SendInt32(ENCODER_GetMmPerCountX1000());
    SERIAL_SendString(",SAMPLE_MS,");
    SERIAL_SendInt32(ENCODER_GetSamplePeriodMs());
    SERIAL_SendString("\r\n");
}

static void send_pid_header(void)
{
    /* 流式日志 / 离线日志共用的 CSV 表头 */
    SERIAL_SendString("CSV,time_ms,mode,left_target,right_target,left_mmps,right_mmps,left_pwm,right_pwm,left_error,right_error,count_diff,line_error,line_mask,line_valid,yaw_x10,corner_count,square_state,turn_travel_mm\r\n");
}

static void send_stream_sample(uint32_t nowMs)
{
    /* 周期输出当前状态一行。
     * elapsed 是相对当前模式开始的时间；IDLE 时强制 0 方便画图。 */
    /* 一帧 PID 日志最多约 180 字节。空间不够就整帧跳过，绝不从中间
     * 丢字节，否则无线端会把半条 PID 与下一条文本粘成乱码。 */
    if (SERIAL_TxCanAccept(220U) == 0U) {
        return;
    }

    int16_t leftSpeed = ENCODER_GetLeftSpeed();
    int16_t rightSpeed = ENCODER_GetRightSpeed();
    int32_t elapsed =
        (g_mode == LAB_MODE_IDLE) ? 0 : (int32_t)(nowMs - g_modeStartMs);

    SERIAL_SendString("PID,");
    SERIAL_SendInt32(elapsed);
    SERIAL_SendString(",");
    SERIAL_SendString(mode_name());
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_leftTarget);
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_rightTarget);
    SERIAL_SendString(",");
    SERIAL_SendInt32(leftSpeed);
    SERIAL_SendString(",");
    SERIAL_SendInt32(rightSpeed);
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_leftPwm);
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_rightPwm);
    SERIAL_SendString(",");
    SERIAL_SendInt32((int32_t)g_leftTarget - leftSpeed);
    SERIAL_SendString(",");
    SERIAL_SendInt32((int32_t)g_rightTarget - rightSpeed);
    SERIAL_SendString(",");
    SERIAL_SendInt32(ENCODER_GetLeftCount() - ENCODER_GetRightCount());
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_lineError);
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_lineMask);
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_lineValid);
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_turnYawX10);
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_squareCornerCount);
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_squareState);
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_turnTravelMm);
    SERIAL_SendString("\r\n");
}

/* 循迹自动整定只需要这些字段。完整 PID 行有 19 列，在 50 Hz 下会把
 * UART->ESP-NOW 桥拆成大量小包；短帧保留评分、转弯和安全检查所需数据。 */
static void send_compact_tune_sample(uint32_t nowMs)
{
    SERIAL_SendString("LT,");
    SERIAL_SendInt32((int32_t)(nowMs - g_modeStartMs));
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_leftTarget);
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_rightTarget);
    SERIAL_SendString(",");
    SERIAL_SendInt32(ENCODER_GetLeftSpeed());
    SERIAL_SendString(",");
    SERIAL_SendInt32(ENCODER_GetRightSpeed());
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_lineError);
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_lineMask);
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_lineValid);
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_turnYawX10);
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_squareCornerCount);
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_squareState);
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_turnTravelMm);
    SERIAL_SendString("\r\n");
}

static void capture_run_sample(uint32_t nowMs)
{
    /* 把当前状态写进离线日志缓冲区。
     * 缓冲满或者没有开启 captureLog 时直接返回。 */
    LabLogSample_t *sample;
    int16_t leftSpeed;
    int16_t rightSpeed;

    if ((g_captureLog == 0U) || (g_runLogCount >= LAB_LOG_CAPACITY)) {
        return;
    }

    sample = &g_runLog[g_runLogCount];
    leftSpeed = ENCODER_GetLeftSpeed();
    rightSpeed = ENCODER_GetRightSpeed();

    sample->timeMs = (uint16_t)clamp_i32(
        (int32_t)(nowMs - g_modeStartMs), 0, 65535);
    sample->leftTarget = g_leftTarget;
    sample->rightTarget = g_rightTarget;
    sample->leftSpeed = leftSpeed;
    sample->rightSpeed = rightSpeed;
    sample->leftPwm = g_leftPwm;
    sample->rightPwm = g_rightPwm;
    sample->leftError = (int16_t)(g_leftTarget - leftSpeed);
    sample->rightError = (int16_t)(g_rightTarget - rightSpeed);
    sample->countDiff = clamp_i16(
        ENCODER_GetLeftCount() - ENCODER_GetRightCount(),
        -32768, 32767);
    g_runLogCount++;
}

static void send_stored_sample(const LabLogSample_t *sample)
{
    /* 把离线日志里的一条样本按 PID, 行格式发出，缺失字段补 0 以保持列数一致 */
    SERIAL_SendString("PID,");
    SERIAL_SendInt32(sample->timeMs);
    SERIAL_SendString(",RUN,");
    SERIAL_SendInt32(sample->leftTarget);
    SERIAL_SendString(",");
    SERIAL_SendInt32(sample->rightTarget);
    SERIAL_SendString(",");
    SERIAL_SendInt32(sample->leftSpeed);
    SERIAL_SendString(",");
    SERIAL_SendInt32(sample->rightSpeed);
    SERIAL_SendString(",");
    SERIAL_SendInt32(sample->leftPwm);
    SERIAL_SendString(",");
    SERIAL_SendInt32(sample->rightPwm);
    SERIAL_SendString(",");
    SERIAL_SendInt32(sample->leftError);
    SERIAL_SendString(",");
    SERIAL_SendInt32(sample->rightError);
    SERIAL_SendString(",");
    SERIAL_SendInt32(sample->countDiff);
    SERIAL_SendString(",0,0,0,0,0,0,0");
    SERIAL_SendString("\r\n");
}

static void start_dump(uint32_t nowMs)
{
    /* 开始回放离线日志：
     *   - 重置下标
     *   - 打印总条数
     *   - 打印表头
     * 后续由 update_dump() 周期性把样本按节奏发完 */
    g_dumpIndex = 0U;
    g_dumpActive = 1U;
    g_lastDumpMs = nowMs;
    SERIAL_SendString("DUMP START,");
    SERIAL_SendInt32(g_runLogCount);
    SERIAL_SendString("\r\n");
    send_pid_header();
}

static void update_dump(uint32_t nowMs)
{
    /* 周期性把一条样本发出去；全部发完时打印 DUMP DONE */
    if ((g_dumpActive == 0U) ||
        ((nowMs - g_lastDumpMs) < LAB_DUMP_PERIOD_MS)) {
        return;
    }
    g_lastDumpMs = nowMs;

    if (g_dumpIndex < g_runLogCount) {
        send_stored_sample(&g_runLog[g_dumpIndex]);
        g_dumpIndex++;
    } else {
        g_dumpActive = 0U;
        SERIAL_SendString("DUMP DONE\r\n");
    }
}

static void start_closed_test(LabMode_t mode, int16_t speed,
                              uint32_t durationMs, uint32_t nowMs,
                              uint8_t liveStream, uint8_t captureLog);

static void apply_open_pwm(char wheel, int16_t pwm, uint32_t nowMs)
{
    /* 启动开环 PWM 模式：
     *   L = 只动左轮，R = 只动右轮，B = 两轮同向
     * 首次进入模式时先 stop_motors() 清理状态；上位机为保持
     * 测试而续发同一命令时不先停车，避免 PWM 波形出现周期性断点。 */
    if (g_mode != LAB_MODE_OPEN_PWM) {
        stop_motors();
    }
    g_straightArmed = 0U;
    g_startPending = 0U;
    g_mode = LAB_MODE_OPEN_PWM;

    /* 每条命令都先清两轮逻辑值，切换 L/R/B 时不会遗留上一路 PWM。 */
    g_leftPwm = 0;
    g_rightPwm = 0;

    if (wheel == 'L') {
        g_leftPwm = pwm;
    } else if (wheel == 'R') {
        g_rightPwm = pwm;
    } else {
        g_leftPwm = pwm;
        g_rightPwm = pwm;
    }

    g_modeStartMs = nowMs;
    g_lastMotorCommandMs = nowMs;
    MOTOR_SetPWM(g_leftPwm, g_rightPwm);
    SERIAL_SendString("OK PWM,STBY,");
    SERIAL_SendString((MOTOR_IsDriverEnabled() != 0U) ? "HIGH" : "LOW");
    SERIAL_SendString("\r\n");
}

static void start_tune_metrics(LabTuneKind_t kind, char wheel,
                               int16_t commandValue, int16_t targetSpeed,
                               uint32_t durationMs, uint32_t token,
                               uint32_t nowMs)
{
    /* 新测试一启动就使旧结果失效；TUNEGET 只可能取回本轮完成的数据。 */
    g_lastTuneResultValid = 0U;
    g_tuneMetrics.kind = kind;
    g_tuneMetrics.wheel = wheel;
    g_tuneMetrics.commandValue = commandValue;
    g_tuneMetrics.targetSpeed = targetSpeed;
    g_tuneMetrics.startMs = nowMs;
    g_tuneMetrics.durationMs = durationMs;
    g_tuneMetrics.nextSampleMs = nowMs;
    g_tuneMetrics.sampleCount = 0U;
    g_tuneMetrics.tailCount = 0U;
    g_tuneMetrics.saturationCount = 0U;
    g_tuneMetrics.token = token;
    g_tuneMetrics.riseTimeMs = durationMs;
    g_tuneMetrics.maxSpeed = 0;
    g_tuneMetrics.sumSpeed = 0;
    g_tuneMetrics.sumAbsError = 0;
    g_tuneMetrics.tailSum = 0;
    g_tuneMetrics.tailSquareSum = 0;
    g_tuneMetrics.flags = 0U;
}

static void start_tune_open(char wheel, int16_t pwm, uint32_t durationMs,
                            uint32_t token, uint32_t nowMs)
{
    apply_open_pwm(wheel, pwm, nowMs);
    g_streamEnabled = 0U;
    g_compactTuneStream = 0U;
    start_tune_metrics(LAB_TUNE_OPEN, wheel, pwm, 0, durationMs, token, nowMs);
}

static void start_tune_step(char wheel, int16_t speed, uint32_t durationMs,
                            uint32_t token, uint32_t nowMs)
{
    g_compactTuneStream = 0U;
    start_closed_test((wheel == 'L') ? LAB_MODE_STEP_LEFT :
                      LAB_MODE_STEP_RIGHT, speed, durationMs, nowMs,
                      0U, 0U);
    start_tune_metrics(LAB_TUNE_STEP, wheel, 0, speed, durationMs, token, nowMs);
}

static void send_tune_open_result(const LabTuneMetrics_t *result)
{
    int32_t average = 0;
    if (SERIAL_TxCanAccept(100U) == 0U) return;
    if (result->sampleCount != 0U) {
        average = (int32_t)(result->sumSpeed / result->sampleCount);
    }
    SERIAL_SendString("TO,");
    SERIAL_SendInt32(result->token);
    SERIAL_SendString(",");
    SERIAL_SendString((result->wheel == 'L') ? "L," : "R,");
    SERIAL_SendInt32(result->commandValue);
    SERIAL_SendString(",");
    SERIAL_SendInt32(average);
    SERIAL_SendString(",");
    SERIAL_SendInt32(result->sampleCount);
    SERIAL_SendString(",");
    SERIAL_SendInt32(result->maxSpeed);
    SERIAL_SendString(",");
    SERIAL_SendInt32(result->flags);
    SERIAL_SendString("\r\n");
}

static void send_tune_step_result(const LabTuneMetrics_t *result)
{
    if (SERIAL_TxCanAccept(160U) == 0U) return;
    SERIAL_SendString("TS,");
    SERIAL_SendInt32(result->token);
    SERIAL_SendString(",");
    SERIAL_SendString((result->wheel == 'L') ? "L," : "R,");
    SERIAL_SendInt32(result->sampleCount);
    SERIAL_SendString(",");
    SERIAL_SendInt32((int32_t)result->sumAbsError);
    SERIAL_SendString(",");
    SERIAL_SendInt32(result->maxSpeed);
    SERIAL_SendString(",");
    SERIAL_SendInt32(result->tailCount);
    SERIAL_SendString(",");
    SERIAL_SendInt32((int32_t)result->tailSum);
    SERIAL_SendString(",");
    SERIAL_SendInt32((int32_t)result->tailSquareSum);
    SERIAL_SendString(",");
    SERIAL_SendInt32((int32_t)result->riseTimeMs);
    SERIAL_SendString(",");
    SERIAL_SendInt32(result->saturationCount);
    SERIAL_SendString(",");
    SERIAL_SendInt32(result->flags);
    SERIAL_SendString("\r\n");
}

static void send_tune_ack(const LabTuneMetrics_t *request)
{
    if (SERIAL_TxCanAccept(64U) == 0U) return;
    SERIAL_SendString("TA,");
    SERIAL_SendInt32(request->token);
    SERIAL_SendString(",");
    SERIAL_SendString((request->kind == LAB_TUNE_OPEN) ? "O," : "S,");
    SERIAL_SendString((request->wheel == 'L') ? "L\r\n" : "R\r\n");
}

static uint8_t tune_request_matches(const LabTuneMetrics_t *request,
                                    LabTuneKind_t kind, char wheel,
                                    uint32_t token)
{
    return ((request->kind == kind) && (request->wheel == wheel) &&
            (request->token == token)) ? 1U : 0U;
}

static void send_tune_error(uint32_t token, const char *reason)
{
    if (SERIAL_TxCanAccept(64U) == 0U) return;
    SERIAL_SendString("TE,");
    SERIAL_SendInt32(token);
    SERIAL_SendString(",");
    SERIAL_SendString(reason);
    SERIAL_SendString("\r\n");
}

static void update_tune_metrics(uint32_t nowMs)
{
    uint32_t elapsed;
    int16_t speed;
    int16_t pwm;
    uint8_t finished = 0U;

    if (g_tuneMetrics.kind == LAB_TUNE_NONE) {
        return;
    }
    elapsed = nowMs - g_tuneMetrics.startMs;
    if (nowMs >= g_tuneMetrics.nextSampleMs) {
        g_tuneMetrics.nextSampleMs = nowMs + LAB_TUNE_SAMPLE_PERIOD_MS;
        speed = (g_tuneMetrics.wheel == 'L') ? ENCODER_GetLeftSpeed() :
                                                ENCODER_GetRightSpeed();
        pwm = (g_tuneMetrics.wheel == 'L') ? g_leftPwm : g_rightPwm;

        if (abs_i16(speed) > g_tuneMetrics.maxSpeed) {
            g_tuneMetrics.maxSpeed = abs_i16(speed);
        }

        if (g_tuneMetrics.kind == LAB_TUNE_OPEN) {
            /* 前 700 ms 只让电机建立稳态，不计入前馈均值。 */
            if (elapsed >= 700U) {
                g_tuneMetrics.sumSpeed += abs_i16(speed);
                g_tuneMetrics.sampleCount++;
            }
            if (abs_i16(speed) >= 2450) {
                g_tuneMetrics.flags |= LAB_TUNE_FLAG_CLIPPED;
            }
            if (abs_i16(speed) >= LAB_TUNE_OPEN_STOP_MMPS) {
                g_tuneMetrics.flags |= LAB_TUNE_FLAG_OVERSPEED;
                finished = 1U;
            }
        } else {
            int16_t error = (int16_t)(g_tuneMetrics.targetSpeed - speed);
            g_tuneMetrics.sampleCount++;
            g_tuneMetrics.sumAbsError += abs_i16(error);
            if ((g_tuneMetrics.riseTimeMs == g_tuneMetrics.durationMs) &&
                (speed >= (int16_t)((g_tuneMetrics.targetSpeed * 9) / 10))) {
                g_tuneMetrics.riseTimeMs = elapsed;
            }
            if (pwm >= (LAB_PWM_CLOSED_LIMIT - 5)) {
                g_tuneMetrics.saturationCount++;
            }
            if (elapsed >= ((g_tuneMetrics.durationMs * 3U) / 4U)) {
                g_tuneMetrics.tailCount++;
                g_tuneMetrics.tailSum += speed;
                g_tuneMetrics.tailSquareSum += (int32_t)speed * speed;
            }
            if (abs_i16(speed) >= LAB_TUNE_STEP_STOP_MMPS) {
                g_tuneMetrics.flags |= LAB_TUNE_FLAG_OVERSPEED;
                finished = 1U;
            }
        }
    }

    if ((elapsed >= g_tuneMetrics.durationMs) || (finished != 0U)) {
        LabTuneKind_t kind = g_tuneMetrics.kind;
        g_lastTuneResult = g_tuneMetrics;
        g_lastTuneResultValid = 1U;
        g_tuneMetrics.kind = LAB_TUNE_NONE;
        stop_motors();
        if (kind == LAB_TUNE_OPEN) {
            send_tune_open_result(&g_lastTuneResult);
        } else {
            send_tune_step_result(&g_lastTuneResult);
        }
    }
}

static void start_closed_test(LabMode_t mode, int16_t speed,
                              uint32_t durationMs, uint32_t nowMs,
                              uint8_t liveStream, uint8_t captureLog)
{
    /* 启动任意一种闭环测试（阶跃 / 直线 / 循迹）：
     *   - 复位编码器、PI 状态
     *   - 切换模式
     *   - 打印测试开始信息 + PID 表头
     * 离线路由 captureLog 决定 */
    stop_motors();
    g_straightArmed = 0U;
    g_startPending = 0U;
    ENCODER_Reset();

    g_mode = mode;
    g_testSpeed = speed;
    g_modeStartMs = nowMs;
    g_modeDurationMs = durationMs;
    g_lastSpeedSequence = ENCODER_GetSpeedSampleSequence();
    g_lastStreamMs = nowMs;
    g_lastLogMs = nowMs;
    g_streamEnabled = liveStream;
    g_captureLog = captureLog;
    g_lineError = 0;
    g_linePreviousError = 0;
    g_lineTurnMmps = 0;
    g_lineLostMs = 0U;
    g_lineValid = 0U;
    g_lineMask = 0U;
    g_lineKickMmps = 0;
    g_lineKickEndMs = 0U;

    if (captureLog != 0U) {
        g_runLogCount = 0U;
        g_dumpIndex = 0U;
        g_dumpActive = 0U;
    }

    SERIAL_SendString("TEST START,");
    SERIAL_SendString(mode_name());
    SERIAL_SendString(",");
    SERIAL_SendInt32(speed);
    SERIAL_SendString(",");
    SERIAL_SendInt32((int32_t)durationMs);
    SERIAL_SendString("\r\n");
    if (liveStream != 0U) {
        send_pid_header();
    }
}

static uint8_t start_square_test(int16_t speed, uint8_t laps,
                                 uint32_t nowMs)
{
    /* 启动方框模式：
     *   - 必须 IMU / 灰度都准备好
     *   - laps 不能为 0 或超过上限
     *   - 清理方框状态机变量 */
    if ((IMU_IsReady() == 0U) || (SENSOR_GetI2cOk() == 0U) ||
        (laps == 0U) || (laps > LAB_SQUARE_MAX_LAPS)) {
        return 0U;
    }

    stop_motors();
    ENCODER_Reset();
    g_mode = LAB_MODE_SQUARE;
    g_testSpeed = speed;
    g_modeStartMs = nowMs;
    g_modeDurationMs = (uint32_t)laps * 60000U;
    g_lastSpeedSequence = ENCODER_GetSpeedSampleSequence();
    g_lastStreamMs = nowMs;
    g_streamEnabled = 1U;
    g_captureLog = 0U;
    g_squareState = SQUARE_STATE_LINE;
    g_squareStateStartMs = nowMs;
    g_squareTurnStartMs = nowMs;
    g_squareTargetLaps = laps;
    g_squareCornerCount = 0U;
    g_cornerArmCount = 0U;
    g_cornerDetectCount = 0U;
    g_turnCenterCount = 0U;
    g_turnYawX10 = 0;
    g_turnLastAcceptedYawX10 = 0;
    g_turnYawReliable = 1U;
    g_turnLineCleared = 0U;
    g_turnLineClearCount = 0U;
    g_turnLineCaptureCount = 0U;
    g_turnStartLeftMm = ENCODER_GetLeftDistanceMm();
    g_turnStartRightMm = ENCODER_GetRightDistanceMm();
    g_turnTravelMm = 0;
    g_turnCapturedByLine = 0U;
    g_edgeStartLeftMm = ENCODER_GetLeftDistanceMm();
    g_edgeStartRightMm = ENCODER_GetRightDistanceMm();
    g_edgeTravelMm = 0;
    g_cornerMinEdgeMm = LAB_EDGE_MIN_INITIAL_MM;
    g_lineError = 0;
    g_linePreviousError = 0;
    g_lineTurnMmps = 0;
    g_lineLostMs = 0U;

    SERIAL_SendString("TEST START,SQUARE,");
    SERIAL_SendInt32(speed);
    SERIAL_SendString(",");
    SERIAL_SendInt32(laps);
    SERIAL_SendString("\r\n");
    if (g_compactTuneStream == 0U) {
        send_pid_header();
    }
    return 1U;
}

static void square_abort(const char *reason)
{
    /* 方框模式异常中止：停机 + 关闭数据流 + 打印原因 */
    stop_motors();
    g_streamEnabled = 0U;
    g_compactTuneStream = 0U;
    SERIAL_SendString("SQUARE ERROR,");
    SERIAL_SendString(reason);
    SERIAL_SendString("\r\n");
}

static void square_begin_turn(uint32_t nowMs)
{
    /* 在走完一条直边后调用：
     *   1. 用刚刚走过的距离更新“最短边长”估计（带上限裁剪）
     *   2. 把方框状态切到 APPROACH 并清零转弯相关计数 */
    int16_t learnedMinimum;

    g_edgeTravelMm = square_edge_travel_mm();
    learnedMinimum = clamp_i16(
        ((int32_t)g_edgeTravelMm * 3L) / 5L,
        LAB_EDGE_MIN_LIMIT_MM,
        LAB_EDGE_MAX_LIMIT_MM);
    if (g_squareCornerCount == 0U) {
        /* 第一条边：直接用观察值当起点 */
        g_cornerMinEdgeMm = learnedMinimum;
    } else {
        /* 之后的边：用 3/4 历史 + 1/4 本次的 IIR 平滑 */
        g_cornerMinEdgeMm = (int16_t)(
            (((int32_t)g_cornerMinEdgeMm * 3L) +
             learnedMinimum + 2L) / 4L);
    }

    g_squareState = SQUARE_STATE_APPROACH;
    g_squareStateStartMs = nowMs;
    g_squareTurnStartMs = nowMs;
    g_cornerArmCount = 0U;
    g_cornerDetectCount = 0U;
    g_turnCenterCount = 0U;
    g_turnYawX10 = 0;
    reset_pi_state();
}

static uint8_t square_complete_corner(uint32_t nowMs)
{
    /* 一次转角完成后的处理：
     *   - 如果是“线触发”且里程合理，把转弯里程 IIR 进 g_turnDistanceMm
     *   - 计数 +1 并打印 TURN DONE
     *   - 达到目标圈数后保存参数、打印 SQUARE LEARNED / DONE
     *   - 否则回到 LINE 状态继续走下一条边 */
    if ((g_turnCapturedByLine != 0U) &&
        (g_turnTravelMm >= 50) &&
        (g_turnTravelMm <= 140)) {
        g_turnDistanceMm = (int16_t)(
            (((int32_t)g_turnDistanceMm * 3L) +
             g_turnTravelMm + 2L) / 4L);
    }

    g_squareCornerCount++;
    SERIAL_SendString("TURN DONE,");
    SERIAL_SendInt32(g_squareCornerCount);
    SERIAL_SendString(",");
    SERIAL_SendInt32(g_turnYawX10);
    SERIAL_SendString(",");
    SERIAL_SendInt32((int32_t)(nowMs - g_squareTurnStartMs));
    SERIAL_SendString("\r\n");

    if (g_squareCornerCount >=
        (uint8_t)(g_squareTargetLaps * 4U)) {
        /* 圈数完成：停机并尝试把学习到的转弯里程写回 Flash */
        stop_motors();
        g_streamEnabled = 0U;
        g_compactTuneStream = 0U;
        SERIAL_SendString("SQUARE LEARNED,TURNDIST,");
        SERIAL_SendInt32(g_turnDistanceMm);
        SERIAL_SendString("\r\n");
        SERIAL_SendString("SQUARE DONE,");
        SERIAL_SendInt32(g_squareCornerCount);
        SERIAL_SendString("\r\n");
        if (save_flash_parameters() != 0U) {
            SERIAL_SendString("SQUARE AUTOSAVE OK\r\n");
        } else {
            SERIAL_SendString("SQUARE AUTOSAVE ERROR\r\n");
        }
        return 1U;
    }

    /* 继续下一条边：清状态、记里程起点 */
    g_squareState = SQUARE_STATE_LINE;
    g_squareStateStartMs = nowMs;
    g_edgeStartLeftMm = ENCODER_GetLeftDistanceMm();
    g_edgeStartRightMm = ENCODER_GetRightDistanceMm();
    g_edgeTravelMm = 0;
    g_cornerArmCount = 0U;
    g_cornerDetectCount = 0U;
    g_turnCenterCount = 0U;
    g_linePreviousError = g_lineError;
    reset_pi_state();
    return 0U;
}

static uint8_t set_parameter(const char *name, int32_t value)
{
    /* 处理 SET xxx value 命令：
     *   - 每种参数都做范围检查，越界直接拒绝
     *   - 设置成功后会清掉 PI 积分，避免新参数叠加旧积分 */
    if (text_equal(name, "LKP") != 0U) {
        if ((value < 0) || (value > 3000)) return 0U;
        g_leftPi.kpX1000 = value;
    } else if (text_equal(name, "LKI") != 0U) {
        if ((value < 0) || (value > 3000)) return 0U;
        g_leftPi.kiX1000 = value;
    } else if (text_equal(name, "RKP") != 0U) {
        if ((value < 0) || (value > 3000)) return 0U;
        g_rightPi.kpX1000 = value;
    } else if (text_equal(name, "RKI") != 0U) {
        if ((value < 0) || (value > 3000)) return 0U;
        g_rightPi.kiX1000 = value;
    } else if (text_equal(name, "LFF") != 0U) {
        if ((value < 0) || (value > 1000)) return 0U;
        g_leftPi.ffX1000 = value;
    } else if (text_equal(name, "RFF") != 0U) {
        if ((value < 0) || (value > 1000)) return 0U;
        g_rightPi.ffX1000 = value;
    } else if (text_equal(name, "LMIN") != 0U) {
        if ((value < 0) || (value > 450)) return 0U;
        g_leftPi.minPwm = (int16_t)value;
    } else if (text_equal(name, "RMIN") != 0U) {
        if ((value < 0) || (value > 450)) return 0U;
        g_rightPi.minPwm = (int16_t)value;
    } else if (text_equal(name, "SYNC") != 0U) {
        if ((value < 0) || (value > 5000)) return 0U;
        g_syncKpX1000 = value;
    } else if (text_equal(name, "BIAS") != 0U) {
        if ((value < -80) || (value > 80)) return 0U;
        g_rightBiasMmps = (int16_t)value;
    } else if (text_equal(name, "LINEKP") != 0U) {
        if ((value < 0) || (value > 20000)) return 0U;
        g_lineKpX1000 = value;
    } else if (text_equal(name, "LINEKD") != 0U) {
        if ((value < 0) || (value > 20000)) return 0U;
        g_lineKdX1000 = value;
    } else if (text_equal(name, "TURNANGLE") != 0U) {
        if ((value < 700) || (value > 1100)) return 0U;
        g_turnAngleX10 = (int16_t)value;
    } else if (text_equal(name, "TURNFAST") != 0U) {
        if ((value < 100) || (value > 300) ||
            (value < g_turnSlowPwm)) return 0U;
        g_turnFastPwm = (int16_t)value;
    } else if (text_equal(name, "TURNSLOW") != 0U) {
        if ((value < 80) || (value > 250) ||
            (value > g_turnFastPwm)) return 0U;
        g_turnSlowPwm = (int16_t)value;
    } else if (text_equal(name, "TURNMARGIN") != 0U) {
        if ((value < 50) || (value > 350)) return 0U;
        g_turnSlowMarginX10 = (int16_t)value;
    } else if (text_equal(name, "TURNEXIT") != 0U) {
        if ((value < 80) || (value > 250)) return 0U;
        g_turnExitMmps = (int16_t)value;
    } else if (text_equal(name, "TURNDIST") != 0U) {
        if ((value < 50) || (value > 140)) return 0U;
        g_turnDistanceMm = (int16_t)value;
    } else {
        return 0U;
    }

    /* 修改完任何一项都清掉积分，防止新参数下出现瞬间大跳变 */
    reset_pi_state();
    return 1U;
}

static void start_or_repeat_tune_open(char wheel, int16_t pwm,
                                      uint32_t durationMs, uint32_t token,
                                      uint32_t nowMs)
{
    if (g_tuneMetrics.kind != LAB_TUNE_NONE) {
        if (tune_request_matches(&g_tuneMetrics, LAB_TUNE_OPEN,
                                 wheel, token) != 0U) {
            send_tune_ack(&g_tuneMetrics);
        } else {
            send_tune_error(token, "BUSY");
        }
        return;
    }

    if ((g_lastTuneResultValid != 0U) &&
        (tune_request_matches(&g_lastTuneResult, LAB_TUNE_OPEN,
                              wheel, token) != 0U)) {
        send_tune_ack(&g_lastTuneResult);
        send_tune_open_result(&g_lastTuneResult);
        return;
    }

    start_tune_open(wheel, pwm, durationMs, token, nowMs);
    send_tune_ack(&g_tuneMetrics);
}

static void start_or_repeat_tune_step(char wheel, int16_t speed,
                                      uint32_t durationMs, uint32_t token,
                                      uint32_t nowMs)
{
    if (g_tuneMetrics.kind != LAB_TUNE_NONE) {
        if (tune_request_matches(&g_tuneMetrics, LAB_TUNE_STEP,
                                 wheel, token) != 0U) {
            send_tune_ack(&g_tuneMetrics);
        } else {
            send_tune_error(token, "BUSY");
        }
        return;
    }

    if ((g_lastTuneResultValid != 0U) &&
        (tune_request_matches(&g_lastTuneResult, LAB_TUNE_STEP,
                              wheel, token) != 0U)) {
        send_tune_ack(&g_lastTuneResult);
        send_tune_step_result(&g_lastTuneResult);
        return;
    }

    start_tune_step(wheel, speed, durationMs, token, nowMs);
    send_tune_ack(&g_tuneMetrics);
}

static void send_set_ack(uint32_t token, const char *name, int32_t value)
{
    if (SERIAL_TxCanAccept(80U) == 0U) return;
    SERIAL_SendString("SA,");
    SERIAL_SendInt32(token);
    SERIAL_SendString(",");
    SERIAL_SendString(name);
    SERIAL_SendString(",");
    SERIAL_SendInt32(value);
    SERIAL_SendString("\r\n");
}

static void send_save_ack(uint32_t token, uint8_t succeeded)
{
    if (SERIAL_TxCanAccept(64U) == 0U) return;
    SERIAL_SendString("SV,");
    SERIAL_SendInt32(token);
    SERIAL_SendString((succeeded != 0U) ? ",OK\r\n" : ",ERR\r\n");
}

static void send_square_ack(uint32_t token, const char *status)
{
    if (SERIAL_TxCanAccept(64U) == 0U) return;
    SERIAL_SendString("SQ,");
    SERIAL_SendInt32(token);
    SERIAL_SendString(",");
    SERIAL_SendString(status);
    SERIAL_SendString("\r\n");
}

static void send_kick_ack(uint32_t token, const char *status)
{
    if (SERIAL_TxCanAccept(64U) == 0U) return;
    SERIAL_SendString("KA,");
    SERIAL_SendInt32(token);
    SERIAL_SendString(",");
    SERIAL_SendString(status);
    SERIAL_SendString("\r\n");
}

/* 解析一条完整 ASCII 命令并执行。命令在此处统一做大小写、空白、参数范围和
 * 模式安全检查；所有会驱动电机的命令最终仍通过受限的控制状态机下发。 */
static void process_command(char *line, uint32_t nowMs)
{
    /* 串口命令统一入口：
     *   1. 整行转大写 -> split_tokens 切分最多 5 个 token
     *   2. 逐个 if 分支匹配命令格式
     *   3. 每种命令都自己做参数合法性检查
     * 注意：调用者要保证 line 是一个可写的、以 \0 结尾的字符串 */
    char *tokens[5];
    uint8_t count;
    int32_t value1;
    int32_t value2;
    int32_t value3;

    normalize_line(line);
    count = split_tokens(line, tokens, 5U);
    if (count == 0U) {
        return;
    }

    /* ---- 基础状态/帮助命令 ---- */
    if ((count == 1U) && (text_equal(tokens[0], "PING") != 0U)) {
        SERIAL_SendString("PONG\r\n");
    } else if ((count == 2U) &&
               (text_equal(tokens[0], "HELLO") != 0U) &&
               (parse_i32(tokens[1], &value1) != 0U) &&
               (value1 >= 0) && (value1 <= LAB_TOKEN_MAX)) {
        send_protocol_hello((uint32_t)value1);
    } else if ((count == 1U) && (text_equal(tokens[0], "HELP") != 0U)) {
        send_help();
    } else if ((count == 1U) && (text_equal(tokens[0], "STOP") != 0U)) {
        /* 强制停机并清空所有 ARM / 离线日志 / 流状态 */
        g_tuneMetrics.kind = LAB_TUNE_NONE;
        stop_motors();
        g_streamEnabled = 0U;
        g_compactTuneStream = 0U;
        g_captureLog = 0U;
        g_dumpActive = 0U;
        g_straightArmed = 0U;
        g_startPending = 0U;
        g_squareStartPending = 0U;
        SERIAL_SendString("OK STOP\r\n");
    } else if ((count == 1U) && (text_equal(tokens[0], "STATUS") != 0U)) {
        send_status(nowMs);
    } else if ((count == 1U) && (text_equal(tokens[0], "SENSOR") != 0U)) {
        send_sensor_status();
    } else if ((count == 1U) && (text_equal(tokens[0], "IMU") != 0U)) {
        send_imu_status();
    } else if ((count == 1U) && (text_equal(tokens[0], "PARAM") != 0U)) {
        send_parameters();
    } else if ((count == 2U) &&
               (text_equal(tokens[0], "SAVE") != 0U) &&
               (parse_i32(tokens[1], &value1) != 0U) &&
               (value1 >= 0) && (value1 <= LAB_TOKEN_MAX) &&
               (g_mode == LAB_MODE_IDLE)) {
        uint32_t saveToken = (uint32_t)value1;
        if ((g_lastSaveTokenValid != 0U) &&
            (g_lastSaveToken == saveToken)) {
            send_save_ack(saveToken, g_lastSaveSucceeded);
        } else {
            MOTOR_Stop();
            g_lastSaveSucceeded = save_flash_parameters();
            g_lastSaveToken = saveToken;
            g_lastSaveTokenValid = 1U;
            send_save_ack(saveToken, g_lastSaveSucceeded);
        }
    } else if ((count == 1U) && (text_equal(tokens[0], "SAVE") != 0U) &&
               (g_mode == LAB_MODE_IDLE)) {
        /* 仅空闲时允许烧写 Flash，避免在电机运行时擦 Flash 出问题 */
        MOTOR_Stop();
        if (save_flash_parameters() != 0U) {
            SERIAL_SendString("OK SAVE\r\n");
        } else {
            SERIAL_SendString("ERR SAVE\r\n");
        }
    } else if ((count == 1U) && (text_equal(tokens[0], "LOAD") != 0U) &&
               (g_mode == LAB_MODE_IDLE)) {
        if (load_flash_parameters() != 0U) {
            SERIAL_SendString("OK LOAD\r\n");
            send_parameters();
        } else {
            SERIAL_SendString("ERR LOAD\r\n");
        }
    } else if ((count == 1U) && (text_equal(tokens[0], "DUMP") != 0U) &&
               (g_mode == LAB_MODE_IDLE)) {
        start_dump(nowMs);
    } else if ((count == 2U) &&
               (text_equal(tokens[0], "STREAM") != 0U) &&
               (text_equal(tokens[1], "ON") != 0U)) {
        /* 实时 PID 数据流 */
        g_streamEnabled = 1U;
        g_compactTuneStream = 0U;
        g_lastStreamMs = nowMs;
        SERIAL_SendString("OK STREAM ON\r\n");
        send_pid_header();
    } else if ((count == 2U) &&
               (text_equal(tokens[0], "STREAM") != 0U) &&
               (text_equal(tokens[1], "TUNE") != 0U)) {
        /* 下一次 LINE/SQUARE 使用循迹整定短帧；此命令本身不启动电机。 */
        g_streamEnabled = 0U;
        g_compactTuneStream = 1U;
        g_lastStreamMs = nowMs;
        SERIAL_SendString("OK STREAM TUNE\r\n");
    } else if ((count == 2U) &&
               (text_equal(tokens[0], "STREAM") != 0U) &&
               (text_equal(tokens[1], "OFF") != 0U)) {
        g_streamEnabled = 0U;
        g_compactTuneStream = 0U;
        SERIAL_SendString("OK STREAM OFF\r\n");
    } else if ((count == 2U) &&
               (text_equal(tokens[0], "TUNEGET") != 0U)) {
        /* 结果帧未到达上位机时可重复读取；不会重新启动电机或修改参数。 */
        if ((parse_i32(tokens[1], &value1) == 0U) ||
            (value1 < 0) || (value1 > LAB_TOKEN_MAX)) {
            SERIAL_SendString("ERR TUNE TOKEN\r\n");
        } else if ((g_tuneMetrics.kind != LAB_TUNE_NONE) &&
                   ((uint32_t)value1 == g_tuneMetrics.token)) {
            SERIAL_SendString("TP,");
            SERIAL_SendInt32(value1);
            SERIAL_SendString("\r\n");
        } else if ((g_lastTuneResultValid == 0U) ||
                   ((uint32_t)value1 != g_lastTuneResult.token)) {
            send_tune_error((uint32_t)value1, "UNKNOWN");
        } else if (g_lastTuneResult.kind == LAB_TUNE_OPEN) {
            send_tune_open_result(&g_lastTuneResult);
        } else if (g_lastTuneResult.kind == LAB_TUNE_STEP) {
            send_tune_step_result(&g_lastTuneResult);
        } else {
            SERIAL_SendString("TUNE PENDING\r\n");
        }
    } else if ((count == 4U) &&
               (text_equal(tokens[0], "KICK") != 0U) &&
               (parse_i32(tokens[1], &value1) != 0U) &&
               (parse_i32(tokens[2], &value2) != 0U) &&
               (parse_i32(tokens[3], &value3) != 0U) &&
               (value1 >= -60) && (value1 <= 60) &&
               (value2 >= 50) && (value2 <= 400) &&
               (value3 >= 0) && (value3 <= LAB_TOKEN_MAX)) {
        uint32_t kickToken = (uint32_t)value3;
        if ((g_lastKickTokenValid != 0U) &&
            (g_lastKickToken == kickToken)) {
            send_kick_ack(kickToken, "OK");
        } else if ((g_mode == LAB_MODE_LINE) ||
                   ((g_mode == LAB_MODE_SQUARE) &&
                    (g_squareState == SQUARE_STATE_LINE))) {
            g_lineKickMmps = (int16_t)value1;
            g_lineKickEndMs = nowMs + (uint32_t)value2;
            g_lastKickToken = kickToken;
            g_lastKickTokenValid = 1U;
            send_kick_ack(kickToken, "OK");
        } else {
            send_kick_ack(kickToken, "ERR");
        }
    } else if ((count == 3U) &&
               (text_equal(tokens[0], "KICK") != 0U) &&
               (parse_i32(tokens[1], &value1) != 0U) &&
               (parse_i32(tokens[2], &value2) != 0U) &&
               (value1 >= -60) && (value1 <= 60) &&
               (value2 >= 50) && (value2 <= 400) &&
               ((g_mode == LAB_MODE_LINE) ||
                ((g_mode == LAB_MODE_SQUARE) &&
                 (g_squareState == SQUARE_STATE_LINE)))) {
        /* 临时给循迹 / 方框直线段加一个左右差速，方便调试小弯 */
        g_lineKickMmps = (int16_t)value1;
        g_lineKickEndMs = nowMs + (uint32_t)value2;
        SERIAL_SendString("OK KICK\r\n");
    } else if ((count == 3U) &&
               (text_equal(tokens[0], "PWM") != 0U) &&
               (parse_i32(tokens[2], &value1) != 0U) &&
               (value1 >= 0) && (value1 <= LAB_PWM_OPEN_LIMIT) &&
               ((text_equal(tokens[1], "L") != 0U) ||
                (text_equal(tokens[1], "R") != 0U) ||
                (text_equal(tokens[1], "B") != 0U))) {
        /* 开环 PWM：L/R/B 选轮 */
        g_tuneMetrics.kind = LAB_TUNE_NONE;
        apply_open_pwm(tokens[1][0], (int16_t)value1, nowMs);
    } else if ((count == 5U) &&
               (text_equal(tokens[0], "TUNEOPEN") != 0U) &&
               ((text_equal(tokens[1], "L") != 0U) ||
                (text_equal(tokens[1], "R") != 0U)) &&
               (parse_i32(tokens[2], &value1) != 0U) &&
               (parse_i32(tokens[3], &value2) != 0U) &&
               (parse_i32(tokens[4], &value3) != 0U) &&
               (value1 >= 0) && (value1 <= LAB_PWM_OPEN_LIMIT) &&
               (value2 >= 1000) && (value2 <= LAB_OPEN_TIMEOUT_MS) &&
               (value3 >= 0) && (value3 <= LAB_TOKEN_MAX)) {
        /* 静默开环辨识：本地取稳态均值，只回传一条 TUNE,OPEN 结果。 */
        start_or_repeat_tune_open(tokens[1][0], (int16_t)value1,
                                  (uint32_t)value2, (uint32_t)value3, nowMs);
    } else if ((count == 5U) &&
               (text_equal(tokens[0], "TUNESTEP") != 0U) &&
               ((text_equal(tokens[1], "L") != 0U) ||
                (text_equal(tokens[1], "R") != 0U)) &&
               (parse_i32(tokens[2], &value1) != 0U) &&
               (parse_i32(tokens[3], &value2) != 0U) &&
               (parse_i32(tokens[4], &value3) != 0U) &&
               (value1 >= LAB_SPEED_MIN_MMPS) &&
               (value1 <= LAB_SPEED_MAX_MMPS) &&
               (value2 >= LAB_TEST_MIN_MS) &&
               (value2 <= LAB_TEST_MAX_MS) &&
               (value3 >= 0) && (value3 <= LAB_TOKEN_MAX)) {
        /* 静默闭环阶跃：本地统计误差、超调、稳态波动和饱和比例。 */
        start_or_repeat_tune_step(tokens[1][0], (int16_t)value1,
                                  (uint32_t)value2, (uint32_t)value3, nowMs);
    } else if ((count == 4U) &&
               (text_equal(tokens[0], "SET") != 0U) &&
               (parse_i32(tokens[2], &value1) != 0U) &&
               (parse_i32(tokens[3], &value2) != 0U) &&
               (value2 >= 0) && (value2 <= LAB_TOKEN_MAX) &&
               (set_parameter(tokens[1], value1) != 0U)) {
        /* 带 token 的 SET 不再附带整行 PARAM，避免每次候选切换都制造拥塞。 */
        send_set_ack((uint32_t)value2, tokens[1], value1);
    } else if ((count == 3U) &&
               (text_equal(tokens[0], "SET") != 0U) &&
               (parse_i32(tokens[2], &value1) != 0U) &&
               (set_parameter(tokens[1], value1) != 0U)) {
        /* 动态修改参数 */
        SERIAL_SendString("OK SET\r\n");
        send_parameters();
    } else if ((count == 4U) &&
               (text_equal(tokens[0], "STEP") != 0U) &&
               ((text_equal(tokens[1], "L") != 0U) ||
                (text_equal(tokens[1], "R") != 0U)) &&
               (parse_i32(tokens[2], &value1) != 0U) &&
               (parse_i32(tokens[3], &value2) != 0U) &&
               (value1 >= LAB_SPEED_MIN_MMPS) &&
               (value1 <= LAB_SPEED_MAX_MMPS) &&
               (value2 >= LAB_TEST_MIN_MS) &&
               (value2 <= LAB_TEST_MAX_MS)) {
        /* 单轮阶跃响应测试 */
        g_tuneMetrics.kind = LAB_TUNE_NONE;
        start_closed_test((tokens[1][0] == 'L') ?
                          LAB_MODE_STEP_LEFT : LAB_MODE_STEP_RIGHT,
                          (int16_t)value1, (uint32_t)value2, nowMs,
                          1U, 0U);
    } else if ((count == 3U) &&
               ((text_equal(tokens[0], "RUN") != 0U) ||
                (text_equal(tokens[0], "LINE") != 0U)) &&
               (parse_i32(tokens[1], &value1) != 0U) &&
               (parse_i32(tokens[2], &value2) != 0U) &&
               (value1 >= LAB_SPEED_MIN_MMPS) &&
               (value1 <= LAB_SPEED_MAX_MMPS) &&
               (value2 >= LAB_TEST_MIN_MS) &&
               (value2 <= LAB_TEST_MAX_MS)) {
        /* RUN = 直线 + 同步外环；LINE = 循迹 */
        start_closed_test((text_equal(tokens[0], "LINE") != 0U) ?
                              LAB_MODE_LINE : LAB_MODE_STRAIGHT,
                          (int16_t)value1,
                          (uint32_t)value2, nowMs, 1U, 0U);
    } else if ((count == 4U) &&
               (text_equal(tokens[0], "SQUARE") != 0U) &&
               (parse_i32(tokens[1], &value1) != 0U) &&
               (parse_i32(tokens[2], &value2) != 0U) &&
               (parse_i32(tokens[3], &value3) != 0U) &&
               (value1 >= LAB_SPEED_MIN_MMPS) &&
               (value1 <= LAB_SQUARE_MAX_SPEED_MMPS) &&
               (value2 >= 1) && (value2 <= LAB_SQUARE_MAX_LAPS) &&
               (value3 >= 0) && (value3 <= LAB_TOKEN_MAX)) {
        uint32_t squareToken = (uint32_t)value3;
        if ((g_lastSquareTokenValid != 0U) &&
            (g_lastSquareToken == squareToken)) {
            send_square_ack(squareToken,
                            (g_lastSquareAccepted != 0U) ? "OK" : "ERR");
        } else if (g_mode != LAB_MODE_IDLE) {
            send_square_ack(squareToken, "BUSY");
        } else {
            g_lastSquareAccepted = start_square_test(
                (int16_t)value1, (uint8_t)value2, nowMs);
            g_lastSquareToken = squareToken;
            g_lastSquareTokenValid = 1U;
            send_square_ack(squareToken,
                            (g_lastSquareAccepted != 0U) ? "OK" : "ERR");
        }
    } else if ((count == 3U) &&
               (text_equal(tokens[0], "SQUARE") != 0U) &&
               (parse_i32(tokens[1], &value1) != 0U) &&
               (parse_i32(tokens[2], &value2) != 0U) &&
               (value1 >= LAB_SPEED_MIN_MMPS) &&
               (value1 <= LAB_SQUARE_MAX_SPEED_MMPS) &&
               (value2 >= 1) &&
               (value2 <= LAB_SQUARE_MAX_LAPS)) {
        /* 方框模式：速度 + 圈数 */
        if (start_square_test((int16_t)value1, (uint8_t)value2,
                              nowMs) == 0U) {
            SERIAL_SendString("ERR SQUARE SENSOR/IMU NOT READY\r\n");
        }
    } else if ((count == 3U) &&
               (text_equal(tokens[0], "ARM") != 0U) &&
               (parse_i32(tokens[1], &value1) != 0U) &&
               (parse_i32(tokens[2], &value2) != 0U) &&
               (value1 >= LAB_SPEED_MIN_MMPS) &&
               (value1 <= LAB_SPEED_MAX_MMPS) &&
               (value2 >= LAB_TEST_MIN_MS) &&
               (value2 <= LAB_TEST_MAX_MS) &&
               (g_mode == LAB_MODE_IDLE)) {
        /* 准备一次“脱机直线测试”：上位机发完 ARM 就可以断线，
         * 小车检测到 SET 按键后再按 ARM_START_DELAY_MS 自动启动 */
        g_armedSpeed = (int16_t)value1;
        g_armedDurationMs = (uint32_t)value2;
        g_straightArmed = 1U;
        g_startPending = 0U;
        g_streamEnabled = 0U;
        g_captureLog = 0U;
        SERIAL_SendString("ARMED: DISCONNECT TTL, PLACE CAR, PRESS SW3\r\n");
    } else {
        SERIAL_SendString("ERR COMMAND; SEND HELP\r\n");
    }
}

/* 方框左转子状态机：依次执行前探、IMU 相对 yaw 清零、快速原地转、减速转向、
 * 停稳并以灰度重新捕线。IMU 可靠时只允许“捕线 / 达到目标角”结束；编码器
 * 行程仅在 yaw 野点失去可信度后作为后备，避免错误里程提前结束一个正常转弯。 */
static uint8_t square_service_turn(uint32_t nowMs)
{
    /* 方框模式下的转弯子状态机，由 update_closed_loop() 周期性调用。
     * 只有在 SQUARE 模式且不在 LINE / EXIT 状态时才会真正进入处理。
     *
     * 状态转移：
     *   APPROACH  (短直行 + 设好基准 yaw)
     *        -> TURN_FAST (高速差速转向)
     *        -> TURN_SLOW (达到 yaw 或里程阈值后降速)
     *        -> EXIT      (线再次出现 / 里程 / yaw 三选一触发结束)
     *
     * 返回 1 表示“本次正在处理转弯”，调用方应直接 return；返回 0 表示可以继续处理直行段。 */
    uint32_t elapsed;
    int16_t yaw;
    int16_t pwm;
    int32_t leftTravel;
    int32_t rightTravel;

    if ((g_mode != LAB_MODE_SQUARE) ||
        (g_squareState == SQUARE_STATE_LINE) ||
        (g_squareState == SQUARE_STATE_EXIT)) {
        return 0U;
    }

    elapsed = nowMs - g_squareStateStartMs;
    if (g_squareState == SQUARE_STATE_APPROACH) {
        /* 短直行：让两轮同步推进一小段，避免在角点上直接扭方向盘 */
        g_leftPwm = g_turnSlowPwm;
        g_rightPwm = g_turnSlowPwm;
        g_leftTarget = g_turnExitMmps;
        g_rightTarget = g_turnExitMmps;
        MOTOR_SetPWM(g_leftPwm, g_rightPwm);

        if (elapsed >= LAB_TURN_APPROACH_MS) {
            if (IMU_IsReady() == 0U) {
                square_abort("IMU NOT READY");
                return 1U;
            }
            /* 重新设置 yaw 参考点，开始累加相对偏航 */
            IMU_BeginRelativeYaw();
            g_turnYawX10 = 0;
            g_turnLastAcceptedYawX10 = 0;
            g_turnYawReliable = 1U;
            g_turnStartLeftMm = ENCODER_GetLeftDistanceMm();
            g_turnStartRightMm = ENCODER_GetRightDistanceMm();
            g_turnTravelMm = 0;
            g_turnCapturedByLine = 0U;
            g_squareState = SQUARE_STATE_TURN_FAST;
            g_squareStateStartMs = nowMs;
        }
        return 1U;
    }

    if (IMU_IsReady() == 0U) {
        square_abort("IMU LOST");
        return 1U;
    }

    /* 计算本次转弯已经走过的平均里程（mm） */
    leftTravel = ENCODER_GetLeftDistanceMm() - g_turnStartLeftMm;
    rightTravel = ENCODER_GetRightDistanceMm() - g_turnStartRightMm;
    if (leftTravel < 0) leftTravel = -leftTravel;
    if (rightTravel < 0) rightTravel = -rightTravel;
    g_turnTravelMm = (int16_t)((leftTravel + rightTravel) / 2L);

    /* 实时采集灰度掩码，用于“线是否已离开 / 重新出现”判定 */
    g_lineMask = (uint8_t)(SENSOR_GetRawMask() &
                           LAB_LINE_SENSOR_VALID_MASK);
    if (g_turnLineCleared == 0U) {
        if (center_line_seen(g_lineMask) == 0U) {
            /* 中心没线：累计“线已离开”次数 */
            if (g_turnLineClearCount < 255U) {
                g_turnLineClearCount++;
            }
        } else {
            g_turnLineClearCount = 0U;
        }
        if (g_turnLineClearCount >= LAB_TURN_LINE_CLEAR_CONFIRM) {
            g_turnLineCleared = 1U;
        }
    } else if ((g_turnTravelMm >=
                (g_turnDistanceMm / 2)) &&
               (center_line_seen(g_lineMask) != 0U)) {
        /* 转弯过半后中心再次看到线：累计“线已回归”次数 */
        if (g_turnLineCaptureCount < 255U) {
            g_turnLineCaptureCount++;
        }
    } else {
        g_turnLineCaptureCount = 0U;
    }

    /* 读 IMU 偏航角，并做野点过滤：
     *   - 单次跳变超过 LAB_TURN_YAW_MAX_STEP_X10 视为不可信 */
    yaw = IMU_GetRelativeYawX10();
    if (yaw < 0) yaw = (int16_t)(-yaw);
    if ((g_turnYawReliable != 0U) &&
        (abs_i16((int16_t)(yaw -
                           g_turnLastAcceptedYawX10)) <=
         LAB_TURN_YAW_MAX_STEP_X10)) {
        g_turnLastAcceptedYawX10 = yaw;
        g_turnYawX10 = yaw;
    } else if (abs_i16((int16_t)(yaw -
                                  g_turnLastAcceptedYawX10)) >
               LAB_TURN_YAW_MAX_STEP_X10) {
        g_turnYawReliable = 0U;
    }
    elapsed = nowMs - g_squareStateStartMs;

    /* 全局超时：超过最大允许时长直接中止 */
    if (elapsed >= LAB_TURN_MAX_MS) {
        square_abort("TURN TIMEOUT");
        return 1U;
    }

    /* 满足以下任一条件就结束本次转弯：
     *   - 至少经过最短时间
     *   - 线重新出现 / 偏航达到目标
     *   - 只有 yaw 已判不可靠时，才允许编码器里程作为后备结束条件
     * 注意：这里对应“原 exit 前的检测”，所以先记完状态再切到 EXIT。 */
    if ((elapsed >= LAB_TURN_MIN_MS) &&
         ((g_turnLineCaptureCount >=
           LAB_TURN_LINE_CAPTURE_CONFIRM) ||
          ((g_turnYawReliable == 0U) &&
           (g_turnTravelMm >= g_turnDistanceMm)) ||
          ((g_turnYawReliable != 0U) &&
           (g_turnYawX10 >= g_turnAngleX10)))) {
        g_turnCapturedByLine =
            (g_turnLineCaptureCount >=
             LAB_TURN_LINE_CAPTURE_CONFIRM) ? 1U : 0U;
        g_squareState = SQUARE_STATE_EXIT;
        g_squareStateStartMs = nowMs;
        g_turnCenterCount = 0U;
        g_linePreviousError = 0;
        g_lineLostMs = 0U;
        /* 停一下，让车体在惯性下自然稳定 */
        g_leftPwm = 0;
        g_rightPwm = 0;
        g_leftTarget = 0;
        g_rightTarget = 0;
        MOTOR_Stop();
        reset_pi_state();
        /* 最后一个角直接完成，避免 EXIT 状态让车多抖一下 */
        if ((uint8_t)(g_squareCornerCount + 1U) >=
            (uint8_t)(g_squareTargetLaps * 4U)) {
            (void)square_complete_corner(nowMs);
        }
        return 1U;
    }

    /* 高速 -> 降速 切换：依据 yaw / 里程 */
    if ((g_squareState == SQUARE_STATE_TURN_FAST) &&
        (elapsed >= LAB_TURN_MIN_MS) &&
        (((g_turnYawReliable != 0U) &&
          (g_turnYawX10 >=
           (int16_t)(g_turnAngleX10 -
                     g_turnSlowMarginX10))) ||
         (g_turnTravelMm >=
          (int16_t)(g_turnDistanceMm -
                    LAB_TURN_DISTANCE_SLOW_MARGIN_MM)))) {
        g_squareState = SQUARE_STATE_TURN_SLOW;
    }

    /* 实际下发差速：左轮反方向、右轮正方向，形成逆时针偏航 */
    pwm = (g_squareState == SQUARE_STATE_TURN_FAST) ?
              g_turnFastPwm : g_turnSlowPwm;
    g_leftPwm = (int16_t)(-pwm);
    g_rightPwm = pwm;
    g_leftTarget = (int16_t)(-pwm);
    g_rightTarget = pwm;
    MOTOR_SetPWM(g_leftPwm, g_rightPwm);
    return 1U;
}

/* 闭环总调度。仅在检测到新的编码器速度序列后运行一次：
 * LINE/SQUARE 先计算灰度 PD 给出的左右速度差，STRAIGHT 使用计数差同步外环，
 * 最后两轮各自通过 PI+前馈换算 PWM。这样可保证积分周期与 10 ms 测速周期一致。 */
static void update_closed_loop(uint32_t nowMs)
{
    /* 主闭环调度函数：
     *   1. 过滤掉 IDLE 模式（非闭环测试）
     *   2. 等编码器给出新一帧速度（以 sequence 号为准，保证每周期只跑一次）
     *   3. 如果方框模式正在转弯，把控制权交给 square_service_turn()
     *   4. 否则按当前模式计算左右目标速度：
     *      - STEP_LEFT / STEP_RIGHT：只让目标速度作用在单轮
     *      - LINE / SQUARE 的直线段：灰度外环 + 高速降噪 + 启动斜坡
     *      - SQUARE 直线状态：还要做“是否准备进入转角”的判定
     *      - SQUARE EXIT 状态：验证线是否重新回到中心
     *      - STRAIGHT：编码器计数差同步外环
     *   5. 把目标速度丢进 PI 控制器得到最终 PWM 并下发给电机 */
    uint32_t sequence = ENCODER_GetSpeedSampleSequence();
    int16_t leftSpeed;
    int16_t rightSpeed;

    /* 非闭环测试模式：直接返回，不做控制 */
    if ((g_mode != LAB_MODE_STEP_LEFT) &&
        (g_mode != LAB_MODE_STEP_RIGHT) &&
        (g_mode != LAB_MODE_STRAIGHT) &&
        (g_mode != LAB_MODE_LINE) &&
        (g_mode != LAB_MODE_SQUARE)) {
        return;
    }

    /* 编码器速度还没更新到本周期：本周期不计算（保持上一拍的输出） */
    if (sequence == g_lastSpeedSequence) {
        return;
    }
    g_lastSpeedSequence = sequence;

    /* 方框模式正在转弯：由子状态机接管，调用方不再做其他处理 */
    if (square_service_turn(nowMs) != 0U) {
        return;
    }

    /* 单轮阶跃：左/右只有一边有目标速度 */
    if (g_mode == LAB_MODE_STEP_LEFT) {
        g_leftTarget = g_testSpeed;
        g_rightTarget = 0;
    } else if (g_mode == LAB_MODE_STEP_RIGHT) {
        g_leftTarget = 0;
        g_rightTarget = g_testSpeed;
    } else if ((g_mode == LAB_MODE_LINE) ||
               (g_mode == LAB_MODE_SQUARE)) {
        /* 循迹 / 方框直线段：基础目标速度 + 灰度外环 */
        int16_t baseTarget = g_testSpeed;
        int16_t maxTurn;
        int16_t errorDelta;
        uint8_t valid;
        uint32_t elapsed = nowMs - g_modeStartMs;

        /* 启动斜坡 + 方框 EXIT 段使用较低的速度穿越转角 */
        if ((g_mode == LAB_MODE_SQUARE) &&
            (g_squareState == SQUARE_STATE_EXIT)) {
            baseTarget = g_turnExitMmps;
        } else if (elapsed < LAB_START_RAMP_MS) {
            /* 启动后前 LAB_START_RAMP_MS 毫秒线性爬升，避免阶跃冲击 */
            baseTarget = (int16_t)(((int32_t)g_testSpeed * elapsed) /
                                   LAB_START_RAMP_MS);
        }

        /* 读取灰度掩码并计算“线居中误差” */
        g_lineMask = (uint8_t)(SENSOR_GetRawMask() &
                               LAB_LINE_SENSOR_VALID_MASK);
        g_lineError = calculate_line_error(g_lineMask, &valid);
        g_lineValid = valid;

        if (g_mode == LAB_MODE_SQUARE) {
            /* 方框模式额外判断：是否已经走到转角 */
            if (g_squareState == SQUARE_STATE_LINE) {
                g_edgeTravelMm = square_edge_travel_mm();
                if (g_edgeTravelMm < g_cornerMinEdgeMm) {
                    /* 边长太短：不能被判为转角，重新累计确认计数 */
                    g_cornerArmCount = 0U;
                    g_cornerDetectCount = 0U;
                } else if (g_cornerArmCount <
                           LAB_CORNER_ARM_CONFIRM) {
                    /* 第一阶段：连续多次看到“线很稳居中” */
                    if (stable_center_line(g_lineMask) != 0U) {
                        g_cornerArmCount++;
                    } else {
                        g_cornerArmCount = 0U;
                    }
                } else {
                    /* 第二阶段：连续多次看到“左侧大块黑” -> 触发转角 */
                    if (left_corner_present(g_lineMask) != 0U) {
                        if (g_cornerDetectCount < 255U) {
                            g_cornerDetectCount++;
                        }
                    } else {
                        g_cornerDetectCount = 0U;
                    }

                    if (g_cornerDetectCount >=
                        LAB_CORNER_DETECT_CONFIRM) {
                        square_begin_turn(nowMs);
                        (void)square_service_turn(nowMs);
                        return;
                    }
                }
            } else if (g_squareState == SQUARE_STATE_EXIT) {
                /* EXIT 状态：等待线回到中心并稳定 */
                uint32_t exitElapsed =
                    nowMs - g_squareStateStartMs;

                if ((exitElapsed >= LAB_TURN_EXIT_MIN_MS) &&
                    (stable_center_line(g_lineMask) != 0U)) {
                    if (g_turnCenterCount < 255U) {
                        g_turnCenterCount++;
                    }
                } else {
                    g_turnCenterCount = 0U;
                }

                if (g_turnCenterCount >= LAB_TURN_CENTER_CONFIRM) {
                    /* 中心线稳定：完成本次转角 */
                    if (square_complete_corner(nowMs) != 0U) {
                        return;
                    }
                    baseTarget = g_testSpeed;
                } else if (exitElapsed >= LAB_TURN_EXIT_MAX_MS) {
                    /* 长时间没找到线：放弃本次方框 */
                    square_abort("LINE NOT CAPTURED");
                    return;
                }
            }
        }

        /* 灰度外环：根据误差计算左右差速 */
        if (valid != 0U) {
            int16_t absoluteError = abs_i16(g_lineError);
            int16_t previousAbsoluteError =
                abs_i16(g_linePreviousError);
            int16_t turnLimit = LAB_LINE_TURN_LIMIT_MMPS;
            int16_t desiredTurn;
            int32_t pGain = g_lineKpX1000;
            int32_t dGain = g_lineKdX1000;
            uint8_t highSpeed =
                (baseTarget >= LAB_LINE_HIGH_SPEED_MMPS) ? 1U : 0U;

            errorDelta = (int16_t)(g_lineError - g_linePreviousError);
            /* 越靠近线 P 越软、越偏离 P 越强（含高速增益） */
            if (absoluteError <= LAB_LINE_CENTER_ERROR) {
                pGain = (pGain * 3L) / 4L;
            } else if (absoluteError >= LAB_LINE_FAR_ERROR) {
                pGain = (highSpeed != 0U) ?
                    ((pGain * 5L) / 4L) : (pGain * 2L);
            }

            /* D 项：误差在收敛就放大、在发散就缩小，避免抖动 */
            if (absoluteError < previousAbsoluteError) {
                dGain = dGain * 2L;
            } else {
                dGain = dGain / 2L;
            }

            /* 高速下收紧输出限幅 */
            if (highSpeed != 0U) {
                turnLimit = LAB_LINE_HIGH_TURN_LIMIT_MMPS;
            }
            desiredTurn = clamp_i16(
                ((pGain * g_lineError) +
                 (dGain * errorDelta)) / 1000L,
                (int16_t)-turnLimit, turnLimit);
            if (highSpeed != 0U) {
                /* 高速下额外做“爬升速率限制”，抑制突变 */
                g_lineTurnMmps = clamp_i16(
                    desiredTurn,
                    (int16_t)(g_lineTurnMmps -
                              LAB_LINE_HIGH_TURN_SLEW_MMPS),
                    (int16_t)(g_lineTurnMmps +
                              LAB_LINE_HIGH_TURN_SLEW_MMPS));
            } else {
                g_lineTurnMmps = desiredTurn;
            }
            g_linePreviousError = g_lineError;
            g_lineLostMs = 0U;
        } else {
            /* 灰度无效：累加“丢线时长”，到阈值就停转 */
            if (g_lineLostMs < 60000U) {
                g_lineLostMs = (uint16_t)(g_lineLostMs + 10U);
            }
            if (g_lineLostMs >= LAB_LINE_LOST_STOP_MS) {
                if (g_mode == LAB_MODE_SQUARE) {
                    square_abort("LINE LOST");
                } else {
                    stop_motors();
                    g_streamEnabled = 0U;
                    SERIAL_SendString("LINE LOST; STOPPED\r\n");
                }
                return;
            }
            if (g_lineLostMs >= LAB_LINE_RECOVERY_MS) {
                /* 丢线早期：降速巡航、修零 */
                g_lineTurnMmps = 0;
                if (baseTarget > LAB_LINE_RECOVERY_MMPS) {
                    baseTarget = LAB_LINE_RECOVERY_MMPS;
                }
            }
        }

        /* KICK 命令：调试用，临时附加固定差速 */
        if ((g_lineKickMmps != 0) &&
            ((int32_t)(g_lineKickEndMs - nowMs) > 0)) {
            g_lineTurnMmps = clamp_i16(
                (int32_t)g_lineTurnMmps + g_lineKickMmps,
                -LAB_LINE_TURN_LIMIT_MMPS,
                LAB_LINE_TURN_LIMIT_MMPS);
        } else {
            g_lineKickMmps = 0;
            g_lineKickEndMs = 0U;
        }

        /* 最终：基础速度 +/- 修正得到左右目标，并限制不会让目标变成负值 */
        maxTurn = (int16_t)(baseTarget - 20);
        if (maxTurn < 0) maxTurn = 0;
        g_lineTurnMmps = clamp_i16(g_lineTurnMmps, -maxTurn, maxTurn);
        g_leftTarget = clamp_i16((int32_t)baseTarget + g_lineTurnMmps,
                                 0, LAB_SPEED_MAX_MMPS);
        g_rightTarget = clamp_i16((int32_t)baseTarget - g_lineTurnMmps,
                                  0, LAB_SPEED_MAX_MMPS);
    } else {
        /* STRAIGHT 模式：编码器计数差同步外环（左右轮按累计脉冲数对齐） */
        int16_t baseTarget = g_testSpeed;
        int32_t countDiff =
            ENCODER_GetLeftCount() - ENCODER_GetRightCount();
        int32_t averageCount =
            (ENCODER_GetLeftCount() + ENCODER_GetRightCount()) / 2L;
        int32_t desiredCountDiff =
            (averageCount * g_rightBiasMmps) / 10000L;
        int32_t syncError = countDiff - desiredCountDiff;
        int16_t correction;
        int16_t correctionLimit;
        int32_t syncGain;
        uint32_t elapsed = nowMs - g_modeStartMs;

        /* 启动斜坡：避免起步瞬间全速 */
        if (elapsed < LAB_START_RAMP_MS) {
            baseTarget = (int16_t)(((int32_t)g_testSpeed * elapsed) /
                                   LAB_START_RAMP_MS);
        }
        /* 修正限幅 = max(目标速度一半, 全局限幅) */
        correctionLimit = (int16_t)(baseTarget / 2);
        if (correctionLimit > LAB_SYNC_LIMIT_MMPS) {
            correctionLimit = LAB_SYNC_LIMIT_MMPS;
        }
        syncGain = g_syncKpX1000;
        /* 起步阶段把同步增益放大一倍，缩短对齐时间 */
        if (elapsed < LAB_START_RAMP_MS) {
            syncGain = clamp_i32(syncGain * 2L, 0, 5000);
        }
        correction = clamp_i16(
            (syncGain * syncError) / 1000L,
            (int16_t)-correctionLimit, correctionLimit);

        g_leftTarget = clamp_i16((int32_t)baseTarget - correction,
                                 0,
                                 LAB_SPEED_MAX_MMPS);
        g_rightTarget = clamp_i16((int32_t)baseTarget + correction,
                                  0,
                                  LAB_SPEED_MAX_MMPS);
    }

    /* PI 控制器：把目标速度换算成 PWM 并下发给左右电机 */
    leftSpeed = ENCODER_GetLeftSpeed();
    rightSpeed = ENCODER_GetRightSpeed();
    g_leftPwm = calculate_pi_pwm(&g_leftPi, g_leftTarget, leftSpeed);
    g_rightPwm = calculate_pi_pwm(&g_rightPi, g_rightTarget, rightSpeed);
    MOTOR_SetPWM(g_leftPwm, g_rightPwm);
}

static uint8_t key_pressed_raw(uint8_t index)
{
    uint32_t value;
    uint32_t pin;

    /* 新底板三个按键分别接 PB23 / PA8 / PB6，按下均为低电平。 */
    if (index == 0U) {
        value = DL_GPIO_readPins(KEY_SW1_PORT, KEY_SW1_PIN);
        pin = KEY_SW1_PIN;
    } else if (index == 1U) {
        value = DL_GPIO_readPins(KEY_SW2_PORT, KEY_SW2_PIN);
        pin = KEY_SW2_PIN;
    } else {
        value = DL_GPIO_readPins(KEY_SW3_PORT, KEY_SW3_PIN);
        pin = KEY_SW3_PIN;
    }
    return ((value & pin) == 0U) ? 1U : 0U;
}

static void stop_from_key(void)
{
    g_tuneMetrics.kind = LAB_TUNE_NONE;
    stop_motors();
    g_streamEnabled = 0U;
    g_compactTuneStream = 0U;
    g_captureLog = 0U;
    g_dumpActive = 0U;
    g_straightArmed = 0U;
    g_startPending = 0U;
    g_squareStartPending = 0U;
    SERIAL_SendString("KEY STOP\r\n");
}

static void handle_key_press(uint8_t index, uint32_t nowMs)
{
    if (index == 0U) {
        /* SW1: idle only, decrease target laps without wrapping. */
        if ((g_mode == LAB_MODE_IDLE) &&
            (g_startPending == 0U) &&
            (g_squareStartPending == 0U) &&
            (g_selectedSquareLaps > 1U)) {
            g_selectedSquareLaps--;
            g_squareCornerCount = 0U;
            SERIAL_SendString("LAPS SELECTED,");
            SERIAL_SendInt32(g_selectedSquareLaps);
            SERIAL_SendString("\r\n");
        }
        return;
    }

    if (index == 1U) {
        /* SW2: idle only, increase target laps up to the safety limit. */
        if ((g_mode == LAB_MODE_IDLE) &&
            (g_startPending == 0U) &&
            (g_squareStartPending == 0U) &&
            (g_selectedSquareLaps < LAB_STANDALONE_MAX_LAPS)) {
            g_selectedSquareLaps++;
            g_squareCornerCount = 0U;
            SERIAL_SendString("LAPS SELECTED,");
            SERIAL_SendInt32(g_selectedSquareLaps);
            SERIAL_SendString("\r\n");
        }
        return;
    }

    /* SW3 is also a local emergency stop/cancel while anything is active. */
    if ((g_mode != LAB_MODE_IDLE) || (g_startPending != 0U) ||
        (g_squareStartPending != 0U)) {
        stop_from_key();
    } else if (g_straightArmed != 0U) {
        /* Preserve the upper-computer ARM workflow, but use SW3 explicitly. */
        g_straightArmed = 0U;
        g_startPending = 1U;
        g_pendingStartMs = nowMs;
        SERIAL_SendString("ARM RUN: START IN 2 SECONDS\r\n");
    } else {
        g_squareStartPending = 1U;
        g_pendingStartMs = nowMs;
        g_squareCornerCount = 0U;
        SERIAL_SendString("STANDALONE SQUARE: START IN 2 SECONDS\r\n");
    }
}

static void update_arm_key(uint32_t nowMs)
{
    uint8_t index;
    uint32_t elapsed;

    if (nowMs == g_lastKeySampleMs) return;
    elapsed = nowMs - g_lastKeySampleMs;
    g_lastKeySampleMs = nowMs;
    if (elapsed > LAB_KEY_DEBOUNCE_MS) elapsed = LAB_KEY_DEBOUNCE_MS;

    for (index = 0U; index < 3U; index++) {
        LabKeyState_t *key = &g_keys[index];
        uint8_t raw = key_pressed_raw(index);

        if (raw == key->rawLast) {
            uint32_t same = (uint32_t)key->sameMs + elapsed;
            key->sameMs = (uint16_t)((same > 60000U) ? 60000U : same);
        } else {
            key->rawLast = raw;
            key->sameMs = 0U;
        }
        if (key->sameMs >= LAB_KEY_DEBOUNCE_MS) key->stable = raw;

        /* Act once on the debounced press edge; release only rearms the key. */
        if ((key->lastStable == 0U) && (key->stable != 0U)) {
            handle_key_press(index, nowMs);
        }
        key->lastStable = key->stable;
    }
}

static void update_pending_start(uint32_t nowMs)
{
    /* 等待 ARM 启动延迟：
     *   - g_startPending 置 1 期间：到达 LAB_ARM_START_DELAY_MS 后启动 RUN
     *   - g_squareStartPending 置 1 期间：到点后尝试脱机方框 */
    if ((g_startPending != 0U) &&
        ((nowMs - g_pendingStartMs) >= LAB_ARM_START_DELAY_MS)) {
        g_startPending = 0U;
        start_closed_test(LAB_MODE_STRAIGHT, g_armedSpeed,
                          g_armedDurationMs, nowMs, 0U, 1U);
    } else if ((g_squareStartPending != 0U) &&
               ((nowMs - g_pendingStartMs) >=
                LAB_ARM_START_DELAY_MS)) {
        g_squareStartPending = 0U;
        if (start_square_test(
                LAB_STANDALONE_SQUARE_SPEED,
                g_selectedSquareLaps,
                nowMs) == 0U) {
            SERIAL_SendString(
                "STANDALONE ERROR: SENSOR/IMU NOT READY\r\n");
        }
    }
}

void LAB_Init(void)
{
    /* 整个 LAB 模块的初始化入口：
     *   1. 把所有可调参数恢复为默认值
     *   2. 尝试从 Flash 装载参数（成功就覆盖默认值）
     *   3. 复位所有运行时状态
     *   4. 读取一次按键的初始电平，避免上电瞬间误判为“按下” */
    uint8_t loaded;

    g_leftPi.kpX1000 = LAB_DEFAULT_LEFT_KP_X1000;
    g_leftPi.kiX1000 = LAB_DEFAULT_LEFT_KI_X1000;
    g_leftPi.ffX1000 = LAB_DEFAULT_LEFT_FF_X1000;
    g_leftPi.minPwm = LAB_DEFAULT_LEFT_MIN_PWM;

    g_rightPi.kpX1000 = LAB_DEFAULT_RIGHT_KP_X1000;
    g_rightPi.kiX1000 = LAB_DEFAULT_RIGHT_KI_X1000;
    g_rightPi.ffX1000 = LAB_DEFAULT_RIGHT_FF_X1000;
    g_rightPi.minPwm = LAB_DEFAULT_RIGHT_MIN_PWM;
    g_syncKpX1000 = LAB_DEFAULT_SYNC_KP_X1000;
    g_rightBiasMmps = LAB_DEFAULT_RIGHT_BIAS_MMPS;
    g_lineKpX1000 = LAB_DEFAULT_LINE_KP_X1000;
    g_lineKdX1000 = LAB_DEFAULT_LINE_KD_X1000;
    g_turnAngleX10 = LAB_DEFAULT_TURN_ANGLE_X10;
    g_turnFastPwm = LAB_DEFAULT_TURN_FAST_PWM;
    g_turnSlowPwm = LAB_DEFAULT_TURN_SLOW_PWM;
    g_turnSlowMarginX10 = LAB_DEFAULT_TURN_SLOW_MARGIN_X10;
    g_turnExitMmps = LAB_DEFAULT_TURN_EXIT_MMPS;
    g_turnDistanceMm = LAB_DEFAULT_TURN_DISTANCE_MM;

    loaded = load_flash_parameters();

    stop_motors();
    g_streamEnabled = 0U;
    g_compactTuneStream = 0U;
    g_modeStartMs = 0U;
    g_lastMotorCommandMs = 0U;
    g_lastStreamMs = 0U;
    g_lastLogMs = 0U;
    g_lastDumpMs = 0U;
    g_runLogCount = 0U;
    g_dumpIndex = 0U;
    g_captureLog = 0U;
    g_dumpActive = 0U;
    g_straightArmed = 0U;
    g_startPending = 0U;
    g_squareStartPending = 0U;
    g_pendingStartMs = 0U;
    g_lineError = 0;
    g_linePreviousError = 0;
    g_lineTurnMmps = 0;
    g_lineLostMs = 0U;
    g_lineValid = 0U;
    g_lineMask = 0U;
    g_lineKickMmps = 0;
    g_lineKickEndMs = 0U;
    g_squareTargetLaps = 1U;
    g_squareCornerCount = 0U;

    /* 把每个按键初始电平作为稳定电平，避免开机时误识别为按下。 */
    for (uint8_t keyIndex = 0U; keyIndex < 3U; keyIndex++) {
        uint8_t raw = key_pressed_raw(keyIndex);
        g_keys[keyIndex].rawLast = raw;
        g_keys[keyIndex].stable = raw;
        g_keys[keyIndex].lastStable = raw;
        g_keys[keyIndex].sameMs = 0U;
    }
    g_lastKeySampleMs = 0U;
    g_selectedSquareLaps = 1U;

    SERIAL_SendString("\r\nPID LAB CLOSED LOOP READY\r\n");
    SERIAL_SendString((loaded != 0U) ?
                      "PARAM SOURCE: FLASH\r\n" :
                      "PARAM SOURCE: DEFAULT\r\n");
    send_help();
    send_parameters();
    SERIAL_SendString("LAPS SELECTED,1\r\n");
}

void LAB_Task(uint32_t nowMs)
{
    /* 主任务入口，由 main 的循环按固定周期调用：
     *   1. 串口命令读取与处理
     *   2. 按键消抖 + ARM 启动延迟
     *   3. 闭环控制
     *   4. 开环 PWM 超时保护
     *   5. 离线日志采样
     *   6. 模式定时结束
     *   7. 实时 PID 流输出
     *   8. 离线日志 DUMP 回放 */
    char line[LAB_LINE_SIZE];

    /* 串口命令：可能一帧收多行，用 while 一次处理完 */
    while (SERIAL_ReadLine(line, sizeof(line)) != 0U) {
        process_command(line, nowMs);
    }

    update_arm_key(nowMs);
    update_pending_start(nowMs);
    update_closed_loop(nowMs);

    /* 自动整定测试在主控本地采样并由此统一结束；必须放在通用 TEST DONE
     * 判断之前，避免实时 CSV 链路参与速度环参数选择。 */
    update_tune_metrics(nowMs);

    /* 开环 PWM 模式如果长时间没有新命令：强制停转，避免失控 */
    if ((g_mode == LAB_MODE_OPEN_PWM) &&
        ((nowMs - g_lastMotorCommandMs) >= LAB_OPEN_TIMEOUT_MS)) {
        stop_motors();
        SERIAL_SendString("SAFE STOP: PWM TIMEOUT\r\n");
    }

    /* 离线日志：按 LAB_STREAM_PERIOD_MS 周期采样一帧 */
    if ((g_captureLog != 0U) &&
        ((nowMs - g_lastLogMs) >= LAB_STREAM_PERIOD_MS)) {
        g_lastLogMs = nowMs;
        capture_run_sample(nowMs);
    }

    /* 闭环测试总时长到点：停机并提示
     * 如果有离线日志，提示上位机可以开始 DUMP */
    if (((g_mode == LAB_MODE_STEP_LEFT) ||
         (g_mode == LAB_MODE_STEP_RIGHT) ||
         (g_mode == LAB_MODE_STRAIGHT) ||
         (g_mode == LAB_MODE_LINE) ||
         (g_mode == LAB_MODE_SQUARE)) &&
        ((nowMs - g_modeStartMs) >= g_modeDurationMs)) {
        uint8_t hadCapturedLog = g_captureLog;

        stop_motors();
        g_streamEnabled = 0U;
        g_captureLog = 0U;
        SERIAL_SendString("TEST DONE\r\n");
        if (hadCapturedLog != 0U) {
            SERIAL_SendString("LOG READY,");
            SERIAL_SendInt32(g_runLogCount);
            SERIAL_SendString("\r\n");
        }
    }

    /* 实时 PID 数据流：仅在显式开启时按周期输出 */
    if ((g_streamEnabled != 0U) &&
        ((nowMs - g_lastStreamMs) >= LAB_STREAM_PERIOD_MS)) {
        g_lastStreamMs = nowMs;
        if (g_compactTuneStream != 0U) {
            send_compact_tune_sample(nowMs);
        } else {
            send_stream_sample(nowMs);
        }
    }

    /* DUMP 期间按节奏逐条发送离线日志 */
    update_dump(nowMs);

}
