#include "Receive_Gimbal.h"

#include "Global_status.h"
#include "User_math.h"
#include "dm_imu.h"
#include "remote_control.h"

typedef void (*CanRxHandler_t)(uint8_t data[8]);

typedef struct
{
    uint16_t id;
    CanRxHandler_t handler;
} CanRxEntry_t;

static void Receive_Control_Mode(uint8_t data[8])
{
    Global.Control.mode = (enum control_mode_e)((int)bytes_to_float(&data[0]));
    Global.Chassis.mode = (enum chassis_mode_e)((int)bytes_to_float(&data[4]));
}

static void Receive_IMU_Attitude(uint8_t data[8])
{
    dm_imu_gimbal.pitch = bytes_to_float(&data[0]);
    dm_imu_gimbal.yaw_cnt = bytes_to_float(&data[4]);
    //dm_imu_gimbal.yaw = dm_imu_gimbal.yaw_cnt;
}

static void Receive_IMU_Gyro(uint8_t data[8])
{
    dm_imu_gimbal.gyro[0] = bytes_to_float(&data[0]);
    dm_imu_gimbal.gyro[2] = bytes_to_float(&data[4]);
}

static void Receive_Trigger_Mode(uint8_t data[8])
{
    Global.Shoot.trigger_mode = (enum trigger_mode_e)((int)bytes_to_float(&data[0]));
}

static void Receive_RC_ch_data(uint8_t data[8])
{
    RC_data.rc.ch[0] = (int16_t)bytes_to_uint16(&data[0]);
    RC_data.rc.ch[1] = (int16_t)bytes_to_uint16(&data[2]);
    RC_data.rc.ch[2] = (int16_t)bytes_to_uint16(&data[4]);
    RC_data.rc.ch[3] = (int16_t)bytes_to_uint16(&data[6]);
    RC_data.online = 100;
}

static void Receive_RC_s_data(uint8_t data[8])
{
    RC_data.rc.s[0] = bytes_to_uint8(&data[0]);
    RC_data.rc.s[1] = bytes_to_uint8(&data[1]);
    RC_data.online = 100;
}

static const CanRxEntry_t GimbalRxTable[] = {
    { CAN_ID_CHASSIS_MODE,         Receive_Control_Mode },
    { CAN_ID_CHASSIS_IMU_ATTITUDE, Receive_IMU_Attitude },
    { CAN_ID_CHASSIS_IMU_GYRO,     Receive_IMU_Gyro     },
    { CAN_ID_SHOOT_TRIGGER_MODE,   Receive_Trigger_Mode },
    { CAN_ID_CHASSIS_RC_CH,        Receive_RC_ch_data   },
    { CAN_ID_CHASSIS_RC_S,         Receive_RC_s_data    },
};

uint8_t Gimbal_CAN_Dispatch(uint16_t id, uint8_t data[8])
{
    for (uint8_t i = 0; i < sizeof(GimbalRxTable) / sizeof(GimbalRxTable[0]); i++)
    {
        if (GimbalRxTable[i].id == id)
        {
            GimbalRxTable[i].handler(data);
            return 1;
        }
    }

    return 0;
}
