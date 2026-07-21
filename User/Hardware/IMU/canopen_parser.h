#ifndef CANOPEN_PARSER_H
#define CANOPEN_PARSER_H

#include "hipnuc_can_common.h"
#include "fdcan.h"

int canopen_parse_frame(const hipnuc_can_frame_t *frame, can_sensor_data_t *data);
void update_hipnuc_data(const FDCAN_RxHeaderTypeDef *rx,uint8_t*rx_data);

extern can_sensor_data_t hipnuc_can ;
#endif
