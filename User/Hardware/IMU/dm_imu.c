#include "dm_imu.h"

#include "fdcan.h"

#include "User_math.h"

#include "dm_imu.h"
#include <string.h>

// can协议
imu_t dm_imu_gimbal;

void imu_init(uint8_t can_id, uint8_t mst_id, FDCAN_HandleTypeDef *hfdcan, imu_t *imu)
{
	imu->can_id = can_id;
	imu->mst_id = mst_id;
	imu->can_handle = hfdcan;
}

void IMU_RequestData(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, uint8_t reg)
{
	FDCAN_TxHeaderTypeDef tx_header;
	uint8_t cmd[4] = {(uint8_t)can_id, (uint8_t)(can_id >> 8), reg, 0xCC};
	tx_header.DataLength = FDCAN_DLC_BYTES_4;
	tx_header.IdType = FDCAN_STANDARD_ID;
	tx_header.TxFrameType = FDCAN_DATA_FRAME;
	tx_header.Identifier = 0x6FF;
	tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	tx_header.MessageMarker = 0x00;

	if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 2)
	{
		HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, cmd);
	}
}

void IMU_UpdateAccel(uint8_t *pData, imu_t *imu)
{
	uint16_t accel[3];

	accel[0] = pData[3] << 8 | pData[2];
	accel[1] = pData[5] << 8 | pData[4];
	accel[2] = pData[7] << 8 | pData[6];

	imu->accel[0] = uint_to_float(accel[0], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
	imu->accel[1] = uint_to_float(accel[1], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
	imu->accel[2] = uint_to_float(accel[2], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
}

void IMU_UpdateGyro(uint8_t *pData, imu_t *imu)
{
	uint16_t gyro[3];

	gyro[0] = pData[3] << 8 | pData[2];
	gyro[1] = pData[5] << 8 | pData[4];
	gyro[2] = pData[7] << 8 | pData[6];

	imu->gyro[0] = uint_to_float(gyro[0], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
	imu->gyro[1] = uint_to_float(gyro[1], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
	imu->gyro[2] = uint_to_float(gyro[2], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
}

void IMU_UpdateEuler(uint8_t *pData, imu_t *imu)
{
	int euler[3];

	euler[0] = pData[3] << 8 | pData[2];
	euler[1] = pData[5] << 8 | pData[4];
	euler[2] = pData[7] << 8 | pData[6];

	imu->pitch = uint_to_float(euler[0], PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
	imu->yaw = uint_to_float(euler[1], YAW_CAN_MIN, YAW_CAN_MAX, 16);
	imu->roll = uint_to_float(euler[2], ROLL_CAN_MIN, ROLL_CAN_MAX, 16);
}

void IMU_UpdateQuaternion(uint8_t *pData, imu_t *imu)
{
	int w = pData[1] << 6 | ((pData[2] & 0xF8) >> 2);
	int x = (pData[2] & 0x03) << 12 | (pData[3] << 4) | ((pData[4] & 0xF0) >> 4);
	int y = (pData[4] & 0x0F) << 10 | (pData[5] << 2) | (pData[6] & 0xC0) >> 6;
	int z = (pData[6] & 0x3F) << 8 | pData[7];

	imu->q[0] = uint_to_float(w, Quaternion_MIN, Quaternion_MAX, 14);
	imu->q[1] = uint_to_float(x, Quaternion_MIN, Quaternion_MAX, 14);
	imu->q[2] = uint_to_float(y, Quaternion_MIN, Quaternion_MAX, 14);
	imu->q[3] = uint_to_float(z, Quaternion_MIN, Quaternion_MAX, 14);
}

void IMU_UpdateData(uint8_t* pData, imu_t *imu)
{
	switch(pData[0])
	{
		case 1:
			IMU_UpdateAccel(pData,imu);
			break;
		case 2:
			IMU_UpdateGyro(pData,imu);
			break;
		case 3:
			IMU_UpdateEuler(pData,imu);
			break;
		case 4:
			IMU_UpdateQuaternion(pData,imu);
			break;
	}
}
/*
		发送指令
*/
static void imu_send_cmd(uint8_t reg_id, uint8_t ac, uint32_t data, imu_t *imu)
{

	if (imu->can_handle == NULL)
		return;

	FDCAN_TxHeaderTypeDef tx_header;

	uint8_t buf[8] = {0xCC, reg_id, ac, 0xDD, 0, 0, 0, 0};
	memcpy(buf + 4, &data, 4);

	tx_header.DataLength = FDCAN_DLC_BYTES_8;
	tx_header.IdType = FDCAN_STANDARD_ID;
	tx_header.TxFrameType = FDCAN_DATA_FRAME;
	tx_header.Identifier = imu->can_id;
	tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	tx_header.MessageMarker = 0x00;

	if (HAL_FDCAN_GetTxFifoFreeLevel(imu->can_handle) > 2)
	{
		HAL_FDCAN_AddMessageToTxFifoQ(imu->can_handle, &tx_header, buf);
	}
}

void imu_write_reg(uint8_t reg_id, uint32_t data, imu_t *imu)
{
	imu_send_cmd(reg_id, CMD_WRITE, data, imu);
}

void imu_read_reg(uint8_t reg_id, imu_t *imu)
{
	imu_send_cmd(reg_id, CMD_READ, 0, imu);
}

void imu_reboot(imu_t *imu)
{
	imu_write_reg(REBOOT_IMU, 0, imu);
}

void imu_accel_calibration(imu_t *imu)
{
	imu_write_reg(ACCEL_CALI, 0, imu);
}

void imu_gyro_calibration(imu_t *imu)
{
	imu_write_reg(GYRO_CALI, 0, imu);
}

void imu_change_com_port(imu_com_port_e port, imu_t *imu)
{
	imu_write_reg(CHANGE_COM, (uint8_t)port, imu);
}

void imu_set_active_mode_delay(uint32_t delay, imu_t *imu)
{
	imu_write_reg(SET_DELAY, delay, imu);
}

// 设置成主动模式
void imu_change_to_active(imu_t *imu)
{
	imu_write_reg(CHANGE_ACTIVE, 1, imu);
}

void imu_change_to_request(imu_t *imu)
{
	imu_write_reg(CHANGE_ACTIVE, 0, imu);
}

void imu_set_baud(imu_baudrate_e baud, imu_t *imu)
{
	imu_write_reg(SET_BAUD, (uint8_t)baud, imu);
}

void imu_set_can_id(uint8_t can_id, imu_t *imu)
{
	imu_write_reg(SET_CAN_ID, can_id, imu);
}

void imu_set_mst_id(uint8_t mst_id, imu_t *imu)
{
	imu_write_reg(SET_MST_ID, mst_id, imu);
}

void imu_save_parameters(imu_t *imu)
{
	imu_write_reg(SAVE_PARAM, 0, imu);
}

void imu_restore_settings(imu_t *imu)
{
	imu_write_reg(RESTORE_SETTING, 0, imu);
}

void imu_request_accel(imu_t *imu)
{
	imu_read_reg(ACCEL_DATA, imu);
}

void imu_request_gyro(imu_t *imu)
{
	imu_read_reg(GYRO_DATA, imu);
}

void imu_request_euler(imu_t *imu)
{
	imu_read_reg(EULER_DATA, imu);
}

void imu_request_quat(imu_t *imu)
{
	imu_read_reg(QUAT_DATA, imu);
}

void IMU_MatchData(imu_t *imu)
{
	imu->request_cnt++;

	// --- 优化后的轮询逻辑 ---
	// 映射关系: 0->ACCEL, 1->GYRO, 2->EULER, 3->QUAT
	// 对应 dm_imu.h 中的枚举: ACCEL_DATA(1), GYRO_DATA(2), EULER_DATA(3), QUAT_DATA(4)
	uint8_t reg_id = (imu->request_cnt % 4) + ACCEL_DATA;

	// 直接调用现有的读寄存器函数
	imu_read_reg(reg_id, imu);
	// ----------------------

	if (imu->request_cnt >= 4000)
	{
		imu->request_cnt = 0;
	}
}
