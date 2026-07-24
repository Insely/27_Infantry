/*
 * @Date: 2025-10-04 11:35:55
 * @LastEditors: hao && (hao@qlu.edu.cn)
 * @LastEditTime: 2025-10-04 19:33:15
 * @FilePath: \Season-26-Code\User\Hardware\IMU\dm_imu.h
 */
#ifndef __DM_IMU_H
#define __DM_IMU_H

#include "stm32h7xx_hal.h"

/*
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

// 强制 1 字节对齐，防止编译器自动填充导致结构体大小与协议不符 
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
*/


//can协议

#define ACCEL_CAN_MAX (58.8f)    // 加速度CAN解码最大值
#define ACCEL_CAN_MIN	(-58.8f)  // 加速度CAN解码最小值
#define GYRO_CAN_MAX	(34.88f)  // 陀螺仪CAN解码最大值
#define GYRO_CAN_MIN	(-34.88f) // 陀螺仪CAN解码最小值
#define PITCH_CAN_MAX	(90.0f)   // pitch角CAN解码最大值
#define PITCH_CAN_MIN	(-90.0f)  // pitch角CAN解码最小值
#define ROLL_CAN_MAX	(180.0f)  // roll角CAN解码最大值
#define ROLL_CAN_MIN	(-180.0f) // roll角CAN解码最小值
#define YAW_CAN_MAX		(180.0f)  // yaw角CAN解码最大值
#define YAW_CAN_MIN 	(-180.0f) // yaw角CAN解码最小值
#define TEMP_MIN			(0.0f)    // 温度解码最小值
#define TEMP_MAX			(60.0f)   // 温度解码最大值
#define Quaternion_MIN	(-1.0f)  // 四元数分量解码最小值
#define Quaternion_MAX	(1.0f)   // 四元数分量解码最大值
#define IMU_MST_ID      (0x11)   // IMU主机CAN响应ID

#define CMD_READ 0
#define CMD_WRITE 1

typedef enum
{
	COM_USB=0,
	COM_RS485,
	COM_CAN,
	COM_VOFA

}imu_com_port_e;

typedef enum
{
	CAN_BAUD_1M=0,
	CAN_BAUD_500K,
	CAN_BAUD_400K,
	CAN_BAUD_250K,
	CAN_BAUD_200K,
	CAN_BAUD_100K,
	CAN_BAUD_50K,
	CAN_BAUD_25K
	
}imu_baudrate_e;

typedef enum 
{
	REBOOT_IMU=0,
	ACCEL_DATA,
	GYRO_DATA,
	EULER_DATA,
	QUAT_DATA,
	SET_ZERO,
	ACCEL_CALI,
	GYRO_CALI,
	MAG_CALI,
	CHANGE_COM,
	SET_DELAY,
	CHANGE_ACTIVE,
	SET_BAUD,
	SET_CAN_ID,
	SET_MST_ID,
	DATA_OUTPUT_SELECTION,
	SAVE_PARAM=254,
	RESTORE_SETTING=255
}reg_id_e;


typedef struct
{
    uint8_t can_id;
	uint8_t mst_id;
	
	FDCAN_HandleTypeDef *can_handle;

	float pitch;
	float roll;
	float yaw;

	float gyro[3];
	float accel[3];
	
	float q[4];

	float cur_temp;

	float last_yaw;
	float yaw_cnt;

    uint32_t request_cnt;


}imu_t;

extern imu_t dm_imu_gimbal;

void imu_init(uint8_t can_id,uint8_t mst_id,FDCAN_HandleTypeDef *hfdcan,imu_t *imu);

void IMU_RequestData(FDCAN_HandleTypeDef* hfdcan,uint16_t can_id,uint8_t reg);
void IMU_UpdateAccel(uint8_t* pData,imu_t *imu);
void IMU_UpdateGyro(uint8_t* pData,imu_t *imu);
void IMU_UpdateEuler(uint8_t* pData,imu_t *imu);
void IMU_UpdateQuaternion(uint8_t* pData,imu_t *imu);
void IMU_UpdateData(uint8_t* pData,imu_t *imu);

void imu_write_reg(uint8_t reg_id,uint32_t data,imu_t *imu);
void imu_read_reg(uint8_t reg_id,imu_t *imu);
void imu_reboot(imu_t *imu);
void imu_accel_calibration(imu_t *imu);
void imu_gyro_calibration(imu_t *imu);
void imu_change_com_port(imu_com_port_e port,imu_t *imu);
void imu_set_active_mode_delay(uint32_t delay,imu_t *imu);
void imu_change_to_active(imu_t *imu);
void imu_change_to_request(imu_t *imu);
void imu_set_baud(imu_baudrate_e baud,imu_t *imu);
void imu_set_can_id(uint8_t can_id,imu_t *imu);
void imu_set_mst_id(uint8_t mst_id,imu_t *imu);
void imu_save_parameters(imu_t *imu);
void imu_restore_settings(imu_t *imu);
void imu_request_accel(imu_t *imu);
void imu_request_gyro(imu_t *imu);
void imu_request_euler(imu_t *imu);
void imu_request_quat(imu_t *imu);
 
void IMU_MatchData(imu_t *imu);

#endif
