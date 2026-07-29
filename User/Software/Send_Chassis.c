#include "Chassis_omni.h"
#include "Gimbal.h"
#include "Global_status.h"

#include "referee_system.h"
#include "supercup.h"
#include "stm32_time.h"
#include "IMU_updata.h"

#include "dm_imu.h"
#include "remote_control.h"

#include "User_math.h"
#include "robot_param.h"


static void Pack_Control_Mode(uint8_t data[8])
{
    float_to_bytes(Global.Control.mode,                 &data[0]);
    float_to_bytes(Global.Chassis.mode,                 &data[4]);
}

static void Pack_IMU_Attitude(uint8_t data[8])
{
    float_to_bytes(dm_imu_gimbal.pitch,                    &data[0]);
    float_to_bytes(dm_imu_gimbal.yaw,                  &data[4]);
}

static void Pack_IMU_Gyro_01(uint8_t data[8])
{
    float_to_bytes(dm_imu_gimbal.gyro[0],                  &data[0]);
    float_to_bytes(dm_imu_gimbal.gyro[1],                  &data[4]);
}

static void Pack_IMU_Gyro_2(uint8_t data[8])
{
    float_to_bytes(dm_imu_gimbal.gyro[2],                  &data[0]);
    float_to_bytes(dm_imu_gimbal.gyro[3],                  &data[4]);
}

static void Pack_RC_ch_data_0_3(uint8_t data[8])
{
    uint16_to_bytes(RC_data.rc.ch[0], &data[0]);
    uint16_to_bytes(RC_data.rc.ch[1], &data[2]);
    uint16_to_bytes(RC_data.rc.ch[2], &data[4]);
    uint16_to_bytes(RC_data.rc.ch[3], &data[6]);
}

static void Pack_RC_ch_data_4(uint8_t data[8])
{
    uint16_to_bytes(RC_data.rc.ch[4], &data[0]);
}

static void Pack_RC_s_data(uint8_t data[8])
{
    uint8_to_bytes(RC_data.rc.s[0], &data[0]);
    uint8_to_bytes(RC_data.rc.s[1], &data[1]);
    int_to_bytes(RC_data.online, &data[2]);
}

static void Pack_RC_key_data(uint8_t data[8])
{
    uint16_to_bytes(RC_data.key.v, &data[0]);
}

static void Pack_RC_mouse_move_data(uint8_t data[8])
{
    uint16_to_bytes(RC_data.mouse.x, &data[0]);
    uint16_to_bytes(RC_data.mouse.y, &data[2]);
    uint16_to_bytes(RC_data.mouse.z, &data[4]);
}

static void Pack_RC_mouse_press_data(uint8_t data[8])
{
    uint8_to_bytes(RC_data.mouse.press_l, &data[0]);
    uint8_to_bytes(RC_data.mouse.press_r, &data[1]);
    uint8_to_bytes(RC_data.mouse.press_mid, &data[2]);
}

static void Pack_TRIGGER_MODE(uint8_t data[8])
{
    float_to_bytes(Global.Shoot.trigger_mode,  &data[0]);    
}

/*  分频调度表 —— IMU 源 1kHz(TIM13), 全速发送
 *                                       div  phase
 *  tick:  0   1   2   3   4   5   6   7   8   9
 *  帧数:  5   4   5   4   5   4   4   5   4   5   */
static const CanTxEntry_t ChassisTxTable[] = {
    /* 1kHz 速度+IMU: 控制环核心 */
    { CAN_ID_CHASSIS_IMU_ATTITUDE, Pack_IMU_Attitude,   1, 0 },
    { CAN_ID_CHASSIS_IMU_GYRO_01,     Pack_IMU_Gyro_01,       1, 0 },
    { CAN_ID_CHASSIS_IMU_GYRO_23,     Pack_IMU_Gyro_2,       1, 0 },
    /* 200Hz 模式/状态, 各自错开 */
    { CAN_ID_CHASSIS_MODE,         Pack_Control_Mode,   5, 0 },
    { CAN_ID_SHOOT_TRIGGER_MODE,   Pack_TRIGGER_MODE,   5, 2 },
    { CAN_ID_CHASSIS_RC_CH,        Pack_RC_ch_data_0_3,     2, 0 },
    { CAN_ID_CHASSIS_RC_CH_4,      Pack_RC_ch_data_4,     2, 1 },
    { CAN_ID_CHASSIS_RC_S,         Pack_RC_s_data,      5, 1 },
    { CAN_ID_CHASSIS_RC_KEY,       Pack_RC_key_data,    2, 1 },
    { CAN_ID_CHASSIS_RC_MOUSE_MOVE, Pack_RC_mouse_move_data,  2, 0 },
    { CAN_ID_CHASSIS_RC_MOUSE_PRESS, Pack_RC_mouse_press_data, 2, 1 },
    /* 200Hz yaw控制附加数据: 速度前馈 + 扫描速度 */
};


void Chassis_CAN_SendAll(void)
{
    static uint8_t tick = 0;
    static enum trigger_mode_e last_trigger_mode = TRIGGER_CLOSE;
    static enum control_mode_e last_control_mode = LOCK;
    static enum chassis_mode_e last_chassis_mode = FLOW;
    static uint8_t last_rc_s0 = 0;
    static uint8_t last_rc_s1 = 0;
    static uint16_t last_rc_key_v = 0;
    static uint8_t last_mouse_press_l = 0;
    static uint8_t last_mouse_press_r = 0;
    static uint8_t last_mouse_press_mid = 0;
    static uint8_t trigger_mode_fast_resend = 0;
    static uint8_t mode_fast_resend = 0;
    static uint8_t rc_s_fast_resend = 0;
    static uint8_t rc_key_fast_resend = 0;
    static uint8_t mouse_press_fast_resend = 0;
    uint8_t buf[8];

    if (Global.Shoot.trigger_mode != last_trigger_mode)
    {
        last_trigger_mode = Global.Shoot.trigger_mode;
        trigger_mode_fast_resend = 5; /* 边沿后连续 5ms 以 1kHz 快速重发 */
    }

    if (Global.Control.mode != last_control_mode || Global.Chassis.mode != last_chassis_mode)
    {
        last_control_mode = Global.Control.mode;
        last_chassis_mode = Global.Chassis.mode;
        mode_fast_resend = 5; /* mode change uses the same fast resend as trigger. */
    }

    if (RC_data.rc.s[0] != last_rc_s0 || RC_data.rc.s[1] != last_rc_s1)
    {
        last_rc_s0 = RC_data.rc.s[0];
        last_rc_s1 = RC_data.rc.s[1];
        rc_s_fast_resend = 5; /* Resend switch changes at 1kHz for 5ms. */
    }

    if (RC_data.key.v != last_rc_key_v)
    {
        last_rc_key_v = RC_data.key.v;
        rc_key_fast_resend = 5; /* Resend key changes at 1kHz for 5ms. */
    }

    if (RC_data.mouse.press_l != last_mouse_press_l ||
        RC_data.mouse.press_r != last_mouse_press_r ||
        RC_data.mouse.press_mid != last_mouse_press_mid)
    {
        last_mouse_press_l = RC_data.mouse.press_l;
        last_mouse_press_r = RC_data.mouse.press_r;
        last_mouse_press_mid = RC_data.mouse.press_mid;
        mouse_press_fast_resend = 5; /* Resend mouse button changes at 1kHz for 5ms. */
    }

    for (uint8_t i = 0; i < sizeof(ChassisTxTable)/sizeof(ChassisTxTable[0]); i++) {
        uint8_t should_send = (tick % ChassisTxTable[i].divider == ChassisTxTable[i].phase);

        if (ChassisTxTable[i].id == CAN_ID_SHOOT_TRIGGER_MODE && trigger_mode_fast_resend > 0) {
            should_send = 1;
        }
        if (ChassisTxTable[i].id == CAN_ID_CHASSIS_MODE && mode_fast_resend > 0) {
            should_send = 1;
        }
        if (ChassisTxTable[i].id == CAN_ID_CHASSIS_RC_S && rc_s_fast_resend > 0) {
            should_send = 1;
        }
        if (ChassisTxTable[i].id == CAN_ID_CHASSIS_RC_KEY && rc_key_fast_resend > 0) {
            should_send = 1;
        }
        if (ChassisTxTable[i].id == CAN_ID_CHASSIS_RC_MOUSE_PRESS && mouse_press_fast_resend > 0) {
            should_send = 1;
        }

        if (should_send) {
            memset(buf, 0, sizeof(buf));
            ChassisTxTable[i].pack(buf);
            if (Fdcanx_SendData(&hfdcan2, ChassisTxTable[i].id, buf, 8) == 0) {
                if (ChassisTxTable[i].id == CAN_ID_SHOOT_TRIGGER_MODE && trigger_mode_fast_resend > 0) {
                    trigger_mode_fast_resend--;
                }
                if (ChassisTxTable[i].id == CAN_ID_CHASSIS_MODE && mode_fast_resend > 0) {
                    mode_fast_resend--;
                }
                if (ChassisTxTable[i].id == CAN_ID_CHASSIS_RC_S && rc_s_fast_resend > 0) {
                    rc_s_fast_resend--;
                }
                if (ChassisTxTable[i].id == CAN_ID_CHASSIS_RC_KEY && rc_key_fast_resend > 0) {
                    rc_key_fast_resend--;
                }
                if (ChassisTxTable[i].id == CAN_ID_CHASSIS_RC_MOUSE_PRESS && mouse_press_fast_resend > 0) {
                    mouse_press_fast_resend--;
                }
            }
        }
    }

    if (++tick >= 10) tick = 0;  /* LCM(1,2,5)=10, 10ms 一个完整周期 */
}
