/**
 * @file  Chassis.h
 * @brief 底盘统一头文件 —— 根据 ROBOT_TYPE 自动选择舵轮 / 全向轮底盘
 */
#ifndef __CHASSIS_DISPATCH_H__
#define __CHASSIS_DISPATCH_H__

#include "robot_param.h"

#if (USE_CHASSIS_HELM != 0)
  #include "Chassis_helm.h"
#elif (USE_CHASSIS_OMNI != 0)
  #include "Chassis_omni.h"
#else
  /* 无底盘模式：提供空接口，避免编译报错 */
  static inline void Chassis_Init(void) {}
  static inline void Chassis_Tasks(void) {}
  static inline void Chassis_SetX(float x) { (void)x; }
  static inline void Chassis_SetY(float y) { (void)y; }
  static inline void Chassis_SetR(float r) { (void)r; }
  static inline void Chassis_SetAccel(float acc) { (void)acc; }
#endif

#endif /* __CHASSIS_DISPATCH_H__ */
