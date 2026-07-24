/*
 * @Author: hao hao@qlu.edu.cn
 * @Date: 2025-08-31 21:36:57
 * @LastEditors: hao && (hao@qlu.edu.cn)
 * @LastEditTime: 2025-10-03 10:06:39
 * @FilePath: \Season-26-Code\User\BSP\CAN_receive_send.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __CAN_RECEIVE_SEND_H__
#define __CAN_RECEIVE_SEND_H__

//#include "cover_headerfile_h.h"
#include "fdcan.h"
extern FDCAN_HandleTypeDef* Get_CanHandle(uint8_t can_bus);

extern void Can_Init(void);

extern uint8_t Fdcanx_SendData(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
extern uint8_t Fdcanx_Receive(FDCAN_HandleTypeDef *hfdcan,	FDCAN_RxHeaderTypeDef *fdcan_RxHeader, uint8_t *buf);

extern void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan);


/* 自定义板间通信CAN ID */
/* 
 */
#define CAN_ID_CHASSIS_MODE          0x101
#define CAN_ID_CHASSIS_IMU_ATTITUDE  0x102
#define CAN_ID_CHASSIS_IMU_GYRO_01      0x103
#define CAN_ID_CHASSIS_IMU_GYRO_23      0x104
#define CAN_ID_SHOOT_TRIGGER_MODE    0x105
#define CAN_ID_CHASSIS_RC_CH         0x106
#define CAN_ID_CHASSIS_RC_CH_4       0x107
#define CAN_ID_CHASSIS_RC_S          0x108
#define CAN_ID_REFEREE_DATA_1        0x091
#define CAN_ID_REFEREE_DATA_2        0x092
#define CAN_ID_REFEREE_DATA_3        0x093

/* CAN 接收路由表类型 */
typedef void (*CanRxHandler_t)(uint8_t data[8]);
typedef struct {
    uint16_t       id;
    CanRxHandler_t handler;
} CanRxEntry_t;

typedef void (*CanTxPack_t)(uint8_t data[8]);
typedef struct {
    uint16_t    id;
    CanTxPack_t pack;
    uint8_t     divider;   /* 分频系数: 1=每tick发, 2=每2tick发, 5=每5tick发 */
    uint8_t     phase;     /* 相位偏移: 0 ~ divider-1, 错开突发 */
} CanTxEntry_t;


#endif /* __CAN_RECEIVE_SEND_H__ */






