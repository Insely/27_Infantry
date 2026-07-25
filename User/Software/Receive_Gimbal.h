#ifndef __RECEIVE_GIMBAL_H__
#define __RECEIVE_GIMBAL_H__

#include "fdcan.h"
#include <stdint.h>

#define CAN_ID_CHASSIS_MODE          0x101
#define CAN_ID_CHASSIS_IMU_ATTITUDE  0x102
#define CAN_ID_CHASSIS_IMU_GYRO_01      0x103
#define CAN_ID_CHASSIS_IMU_GYRO_2      0x104
#define CAN_ID_SHOOT_TRIGGER_MODE    0x105
#define CAN_ID_CHASSIS_RC_CH_0_3         0x106
#define CAN_ID_CHASSIS_RC_CH_4       0x107
#define CAN_ID_CHASSIS_RC_S          0x108

uint8_t Gimbal_CAN_Dispatch(uint16_t id, uint8_t data[8]);

#endif
