#include "Gimbal.h"
#include "Global_status.h"
#include "remote_control.h"
#include "Auto_control.h"
#include "Vofa_Justfloat_Send.h"

#include "User_math.h"

#include "IMU_updata.h"
#include "dm_imu.h"
#include "USB_VirCom.h"
#include "Chassis_omni.h"

Gimbal_t Gimbal;

static bool ReadyCheck(float yaw_pos)
{
    static int time;
    static int total_time; // 总计时，用于超时强制通过

    Gimbal.yaw_location_set = Gimbal.yaw_location_now + (yaw_pos - GIMBALMotor_get_data(YAWMotor).motor_data.para.pos) * RAD_TO_DEG;

    float d_yaw = fabsf(GIMBALMotor_get_data(YAWMotor).motor_data.para.pos - yaw_pos);

    total_time++;

    if (d_yaw < 0.1)
        time++;
    else
        time = 0;
    if (time < 100 && total_time < 3000) // 最多等3秒，超时强制通过
        return false;
    else
        return true;
}
/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param          none
 * @retval         none
 */
void Gimbal_Init()
{
    // 云台电机初始化
    GIMBALMotor_init(GIMBAL_YAW_MOTOR_TYPE, YAWMotor);
    GIMBALMotor_init(GIMBAL_PITCH_MOTOR_TYPE, PITCHMotor);
    /*PID速度环初始化*/
    // 遥控
    PID_Set(&Gimbal.yaw_speed_pid, 500.0f, 0.0f, 10.0f, GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    // 自瞄
    PID_Set(&Gimbal.yaw_auto_speed_pid, 200.0f, 0.0f, 50.0f, GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    /*PID位置环初始化*/
    // 遥控
    PID_Set(&Gimbal.yaw_location_pid, 14.0f, 0.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 100);
    // 自瞄
    PID_Set(&Gimbal.yaw_auto_location_pid, 15.0f, 0.03f, 1.0f, GIMBALMOTOR_MAX_CURRENT, 100);
    // 上电进入纠偏状态，等待云台到位
    Gimbal.State = RIGHTING;
}

/*-------------------- Update --------------------*/

/**
 * @brief          控制量更新（包括状态量和目标量）
 * @param          none
 * @retval         none
 */
void Gimbal_Updater()
{
    /*------状态量更新------*/
    // 速度
    Gimbal.yaw_speed_now = (cos(DEG_TO_RAD * GIMBAL_IMU_DATA.pitch) * GIMBAL_IMU_DATA.gyro[2] - sin(DEG_TO_RAD * GIMBAL_IMU_DATA.pitch) * GIMBAL_IMU_DATA.gyro[0]);
    // 位置
    //  角度多圈处理
    float angle_diff;
    static float last_yaw;
    static uint8_t yaw_init = 0;
    // 首次初始化，避免上电跳变
    if (!yaw_init)
    {
        last_yaw = GIMBAL_IMU_DATA.yaw;
        yaw_init = 1;
    }
    // 计算角度差并处理2π跳变
    angle_diff = GIMBAL_IMU_DATA.yaw - last_yaw;
    // 处理角度跳变（使用180作为阈值）
    if (angle_diff > 180)
    {
        angle_diff -= 360;
    }
    else if (angle_diff < -180)
    {
        angle_diff += 360;
    }
    Gimbal.yaw_location_now += angle_diff;
    last_yaw = GIMBAL_IMU_DATA.yaw;
    /*------目标量更新------*/
    Gimbal.yaw_location_set = Global.Gimbal.input.yaw;
}

/*-------------------- Calculate --------------------*/

/**
 * @brief          控制量解算
 * @param          none
 * @retval         none
 */
#define PITCH_VEL_FF_GAIN (0.65f)  // pitch速度前馈增益: 1.0=直接跟随目标pitch角速度，单位匹配时取小
#define PITCH_ACC_FF_GAIN (0.013f) // pitch加速度前馈增益 增大可更快跟上但易超
#define PITCH_VEL_FF_LIMIT (0.20f)
#define PITCH_ACC_FF_LIMIT (0.15f)
#define PITCH_FF_TOTAL_LIMIT (0.30f)

void Gimbal_Calculater()
{
    static uint8_t last_auto_active = 0;
    if ((Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE || Global.Auto.input.control_mode == 0) && (Global.Gimbal.mode == NORMAL || Global.Gimbal.mode == SHOOT))
    {
        // 切换到遥控模式跳变处理
        if (last_auto_active)
        {
            Gimbal.yaw_location_set = Gimbal.yaw_location_now;
            Global.Gimbal.input.yaw = Gimbal.yaw_location_set;
        }
        last_auto_active = 0;

        //
        Gimbal.yaw_speed_set = PID_Cal(&Gimbal.yaw_location_pid, Gimbal.yaw_location_now, Gimbal.yaw_location_set) * DEG_TO_RAD - IMU_data.gyro[2];

        if (Global.Auto.input.Auto_control_online > 0)
            Global.Auto.input.Auto_control_online--;
    }
    else
    {
        // 自瞄
        Gimbal.yaw_speed_set = PID_Cal(&Gimbal.yaw_auto_location_pid, Gimbal.yaw_location_now, Gimbal.yaw_location_set) * DEG_TO_RAD + Global.Auto.input.yaw_ff - IMU_data.gyro[2];

        Global.Auto.input.Auto_control_online--;
        if (Global.Auto.input.control_mode == 0)
        {
            last_auto_active = 1;
        }
        else
        {
            last_auto_active = 0;
        }
    }
}

/*-------------------- Control --------------------*/

/**
 * @brief          电流值设置
 * @param          none
 * @retval         none
 */
void Gimbal_Controller()
{
    if (Global.Control.mode != LOCK)
    {
        GIMBALMotor_set(YAWMotor, 0, Gimbal.yaw_speed_set, 0, 0, 2.3f);
    }
    else
    {
        GIMBALMotor_set(YAWMotor, 0, 0, 0, 0, 0);
    }
}

/*-------------------- Task --------------------*/

/**
 * @brief          云台任务
 * @param          none
 * @retval         none
 */
void Gimbal_Tasks(void)
{
#if (USE_GIMBAL != 0)

    // 云台数据更新
    Gimbal_Updater();

    if (Gimbal.State != NORMALLY)
    {
        // 上电纠偏阶段：驱动云台回到零位
        if (ReadyCheck(0))
        {
            Gimbal.State = NORMALLY;
            // 同步全局输入为当前位置，防止切换到正常控制时跳变
            Global.Gimbal.input.yaw = Gimbal.yaw_location_now;
        }
        Gimbal_Calculater();
        // 纠偏阶段强制输出，不受 LOCK 模式影响
        GIMBALMotor_set(YAWMotor, 0, Gimbal.yaw_speed_set, 0, 0, 0.5f);
    }
    else
    {
        // 正常控制
        Gimbal_Calculater();
        Gimbal_Controller();
    }

    //Vofa+打印数据
    static uint16_t vofa_cnt = 0;
    float vofa_gimbal_data[2] = {0.0f, 0.0f};

    if (++vofa_cnt >= 10)
    {
        vofa_cnt = 0;
        vofa_gimbal_data[0] = Gimbal.yaw_location_now;
        vofa_gimbal_data[1] = Gimbal.yaw_location_set;
        Vofa_SendFloat(vofa_gimbal_data, 2);
    }
#endif
}

/*-------------------- Set --------------------*/
/**
 * @brief 设置云台PITCHI轴角度
 *
 * @param angle 云台PITCHI轴角度
 */
void Gimbal_SetPitchAngle(float angle)
{
    if (angle < PITCHI_MIN_ANGLE)
        angle = PITCHI_MIN_ANGLE;
    if (angle > PITCHI_MAX_ANGLE)
        angle = PITCHI_MAX_ANGLE;
    Global.Gimbal.input.pitch = angle;
}

/**
 * @brief 设置云台YAW轴角度
 *
 * @param angle 云台YAW轴角度
 */
void Gimbal_SetYawAngle(float angle)
{
    Global.Gimbal.input.yaw = angle;
}
