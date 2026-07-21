/*
 * @Date: 2025-10-04 11:35:55
 * @LastEditors: hao && (hao@qlu.edu.cn)
 * @LastEditTime: 2025-10-04 19:33:15
 * @FilePath: \Season-26-Code\User\Hardware\IMU\dm_imu.h
 */
#ifndef __DM_IMU_H
#define __DM_IMU_H

#include "stm32h7xx_hal.h"

//rs485协议
typedef struct
{
    float accel[3];
    float gyro[3];
    float roll;
    float pitch;
    float yaw;
    float quaternion[4];
    uint32_t update_cnt; // 数据更新计数器，用于在线检测
} dm_imu_t;

extern dm_imu_t dm_imu_data;

/* 强制 1 字节对齐，防止编译器自动填充导致结构体大小与协议不符 */
#pragma pack(push, 1)

// 19字节常规数据帧 (加速度 0x01、角速度 0x02、欧拉角 0x03)
typedef struct
{
    uint8_t header;   // 0x55
    uint8_t tag;
    uint8_t slave_id;
    uint8_t reg;      // 寄存器ID
    float data[3];    // 3个float数据
    uint16_t crc;
    uint8_t tail;     // 0x0A
} normal_packet_t;

// 23字节扩展数据帧 (四元数 0x04)
typedef struct
{
    uint8_t header;   // 0x55
    uint8_t tag;
    uint8_t slave_id;
    uint8_t reg;      // 0x04
    float data[4];    // 4个float数据
    uint16_t crc;
    uint8_t tail;     // 0x0A
} normal_ext_packet_t;

#pragma pack(pop)

// 外部调用接口
void DM_IMU_RS485_Decode(uint8_t* pData, uint16_t len);


/* can协议

#define ACCEL_CAN_MAX (58.8f)
#define ACCEL_CAN_MIN	(-58.8f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN			(0.0f)
#define TEMP_MAX			(60.0f)
#define Quaternion_MIN	(-1.0f)
#define Quaternion_MAX	(1.0f)
#define IMU_MST_ID      (0x11)

typedef struct
{
	float pitch;
	float roll;
	float yaw;

	float gyro[3];
	float accel[3];
	
	float q[4];

	float cur_temp;

}imu_t;

extern imu_t imu ;

void IMU_UpdateData(uint8_t* pData);
void IMU_RequestData(FDCAN_HandleTypeDef* hfdcan,uint16_t can_id,uint8_t reg);
 */

#endif