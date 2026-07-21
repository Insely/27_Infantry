#include "Chassis_omni.h"
#include "Gimbal.h"
#include "Global_status.h"

#include "referee_system.h"
#include "supercup.h"
#include "stm32_time.h"
#include "IMU_updata.h"

#include "dm_imu.h"

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
    float_to_bytes(dm_imu_gimbal.yaw_cnt,                  &data[4]);
}

static void Pack_IMU_Gyro(uint8_t data[8])
{
    float_to_bytes(dm_imu_gimbal.gyro[0],                  &data[0]);
    float_to_bytes(dm_imu_gimbal.gyro[2],                  &data[4]);
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
    { CAN_ID_CHASSIS_IMU_GYRO,     Pack_IMU_Gyro,       1, 0 },
    /* 200Hz 模式/状态, 各自错开 */
    { CAN_ID_CHASSIS_MODE,         Pack_Control_Mode,   5, 0 },
    { CAN_ID_SHOOT_TRIGGER_MODE,   Pack_TRIGGER_MODE,   5, 2 },
    /* 200Hz yaw控制附加数据: 速度前馈 + 扫描速度 */
};


void Chassis_CAN_SendAll(void)
{
    static uint8_t tick = 0;
    static enum trigger_mode_e last_trigger_mode = TRIGGER_CLOSE;
    static enum control_mode_e last_control_mode = RC;
    static enum chassis_mode_e last_chassis_mode = FLOW;
    static uint8_t trigger_mode_fast_resend = 0;
    static uint8_t mode_fast_resend = 0;
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
        mode_fast_resend = 5; /* 模式边沿与 trigger 一样快速补发 */
    }

    for (uint8_t i = 0; i < sizeof(ChassisTxTable)/sizeof(ChassisTxTable[0]); i++) {
        uint8_t should_send = (tick % ChassisTxTable[i].divider == ChassisTxTable[i].phase);

        if (ChassisTxTable[i].id == CAN_ID_SHOOT_TRIGGER_MODE && trigger_mode_fast_resend > 0) {
            should_send = 1;
        }
        if (ChassisTxTable[i].id == CAN_ID_CHASSIS_MODE && mode_fast_resend > 0) {
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
            }
        }
    }

    if (++tick >= 10) tick = 0;  /* LCM(1,2,5)=10, 10ms 一个完整周期 */
}
