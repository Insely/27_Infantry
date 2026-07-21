#include "dm_imu.h"

#include "fdcan.h"

#include "User_math.h"


#include "dm_imu.h"
#include <string.h>

//rs485协议
//dm_imu_t dm_imu_data;

/**
 * @brief 达妙 IMU RS485 协议解析
 * @param pData 串口接收缓冲区指针
 * @param len   本次接收到的数据长度
 */
void DM_IMU_RS485_Decode(uint8_t* pData, uint16_t len)
{
    uint16_t i = 0;
    
    // 遍历整个接收缓冲区
    while (i < len)
    {
        // 寻找帧头 0x55
        if (pData[i] == 0x55)
        {
            uint8_t reg = pData[i + 3]; // 偏移3个字节是寄存器类型

            // 1. 处理 19 字节的常规帧 (加速度、角速度、欧拉角)
            if ((reg == 0x01 || reg == 0x02 || reg == 0x03) && (i + 19 <= len))
            {
                normal_packet_t packet;
                // 内存拷贝，直接映射到结构体
                memcpy(&packet, &pData[i], 19);
                
                // 校验帧尾
                if (packet.tail == 0x0A)
                {
                    if (packet.reg == 0x01) {
                        dm_imu_data.accel[0] = packet.data[0];
                        dm_imu_data.accel[1] = packet.data[1];
                        dm_imu_data.accel[2] = packet.data[2];
                    } else if (packet.reg == 0x02) {
                        dm_imu_data.gyro[0] = packet.data[0];
                        dm_imu_data.gyro[1] = packet.data[1];
                        dm_imu_data.gyro[2] = packet.data[2];
                    } else if (packet.reg == 0x03) {
                        dm_imu_data.roll  = packet.data[0];
                        dm_imu_data.pitch = packet.data[1];
                        dm_imu_data.yaw   = packet.data[2];
                    }
                    dm_imu_data.update_cnt++;
                    i += 19; // 解析成功，跳过这 19 个字节
                    continue;
                }
            }
            // 2. 处理 23 字节的扩展帧 (四元数)
            else if (reg == 0x04 && (i + 23 <= len))
            {
                normal_ext_packet_t ext_packet;
                memcpy(&ext_packet, &pData[i], 23);
                
                // 校验帧尾
                if (ext_packet.tail == 0x0A)
                {
                    dm_imu_data.quaternion[0] = ext_packet.data[0];
                    dm_imu_data.quaternion[1] = ext_packet.data[1];
                    dm_imu_data.quaternion[2] = ext_packet.data[2];
                    dm_imu_data.quaternion[3] = ext_packet.data[3];
                    dm_imu_data.update_cnt++;
                    i += 23; // 解析成功，跳过这 23 个字节
                    continue;
                }
            }
        }
        
        // 如果当前字节不是 0x55，或者帧尾校验不通过，指针步进 1 字节继续找
        i++;
    }
}




/* can协议
imu_t imu;
void IMU_RequestData(FDCAN_HandleTypeDef* hfdcan,uint16_t can_id,uint8_t reg)
{
	FDCAN_TxHeaderTypeDef tx_header;
	uint8_t cmd[4]={(uint8_t)can_id,(uint8_t)(can_id>>8),reg,0xCC};
	tx_header.DataLength=FDCAN_DLC_BYTES_4;
	tx_header.IdType=FDCAN_STANDARD_ID;
	tx_header.TxFrameType=FDCAN_DATA_FRAME;
	tx_header.Identifier=0x6FF;
	tx_header.FDFormat=FDCAN_CLASSIC_CAN;
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;										
	tx_header.MessageMarker = 0x00; 			      

	if(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan)>2)
	{
		HAL_FDCAN_AddMessageToTxFifoQ(hfdcan,&tx_header,cmd);
	}
}


void IMU_UpdateAccel(uint8_t* pData)
{
	uint16_t accel[3];
	
	accel[0]=pData[3]<<8|pData[2];
	accel[1]=pData[5]<<8|pData[4];
	accel[2]=pData[7]<<8|pData[6];
	
	imu.accel[0]=uint_to_float(accel[0],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	imu.accel[1]=uint_to_float(accel[1],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	imu.accel[2]=uint_to_float(accel[2],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	
}

void IMU_UpdateGyro(uint8_t* pData)
{
	uint16_t gyro[3];
	
	gyro[0]=pData[3]<<8|pData[2];
	gyro[1]=pData[5]<<8|pData[4];
	gyro[2]=pData[7]<<8|pData[6];
	
	imu.gyro[0]=uint_to_float(gyro[0],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	imu.gyro[1]=uint_to_float(gyro[1],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	imu.gyro[2]=uint_to_float(gyro[2],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
}


void IMU_UpdateEuler(uint8_t* pData)
{
	int euler[3];
	
	euler[0]=pData[3]<<8|pData[2];
	euler[1]=pData[5]<<8|pData[4];
	euler[2]=pData[7]<<8|pData[6];
	
	imu.pitch=uint_to_float(euler[0],PITCH_CAN_MIN,PITCH_CAN_MAX,16);
	imu.yaw=uint_to_float(euler[1],YAW_CAN_MIN,YAW_CAN_MAX,16);
	imu.roll=uint_to_float(euler[2],ROLL_CAN_MIN,ROLL_CAN_MAX,16);
}


void IMU_UpdateQuaternion(uint8_t* pData)
{
	int w = pData[1]<<6| ((pData[2]&0xF8)>>2);
	int x = (pData[2]&0x03)<<12|(pData[3]<<4)|((pData[4]&0xF0)>>4);
	int y = (pData[4]&0x0F)<<10|(pData[5]<<2)|(pData[6]&0xC0)>>6;
	int z = (pData[6]&0x3F)<<8|pData[7];
	
	imu.q[0] = uint_to_float(w,Quaternion_MIN,Quaternion_MAX,14);
	imu.q[1] = uint_to_float(x,Quaternion_MIN,Quaternion_MAX,14);
	imu.q[2] = uint_to_float(y,Quaternion_MIN,Quaternion_MAX,14);
	imu.q[3] = uint_to_float(z,Quaternion_MIN,Quaternion_MAX,14);
}

void IMU_UpdateData(uint8_t* pData)
{

	switch(pData[0])
	{
		case 1:
			IMU_UpdateAccel(pData);
			break;
		case 2:
			IMU_UpdateGyro(pData);
			break;
		case 3:
			IMU_UpdateEuler(pData);
			break;
		case 4:
			IMU_UpdateQuaternion(pData);
			break;
	}
} */