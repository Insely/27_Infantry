#include "Gimbal.h"
#include "Global_status.h"
#include "remote_control.h"
#include "Auto_control.h"

#include "User_math.h"

#include "IMU_updata.h"
#include "dm_imu.h"
#include "USB_VirCom.h"
#include "Chassis_omni.h"

Gimbal_t Gimbal;
static char buffer[200];
static int send_num;

static bool ReadyCheck(float pitch_pos)
{
    static int time;
    static int total_time; // 总计时，用于超时强制通过

    Gimbal.pitch_location_set = pitch_pos * RAD_TO_DEG;

    float d_pitch = fabsf(Gimbal.pitch_location_now * DEG_TO_RAD - pitch_pos);

    total_time++;

    if (d_pitch < 0.1)
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
    GIMBALMotor_init(GIMBAL_PITCH_MOTOR_TYPE, PITCHMotor);
    /*PID速度环初始化*/
    // 遥控
    // PID_Set(&Gimbal.pitch_speed_pid, 10.0f, 0.0f, 0.0f, 1000000.0f, 1000000.0f);
    // 自瞄
    // PID_Set(&Gimbal.pitch_auto_speed_pid, 1500.0f, 0.0f, 0.0f, 1000000.0f, 1000000.0f);
    /*PID位置环初始化*/
    // 遥控
    PID_Set(&Gimbal.pitch_location_pid, 8.0f, 0.0f, 0.5f, 1718, 1000);
    // 自瞄
    PID_Set(&Gimbal.pitch_auto_location_pid, 5.1f, 0.0f, 0.0f, 1500, 1000);
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
    Gimbal.pitch_speed_now = -(GIMBAL_IMU_DATA.gyro[1]);
    // 位置
    Gimbal.pitch_location_now = -GIMBAL_IMU_DATA.pitch;
    /*------目标量更新------*/
    Gimbal.pitch_location_set = Global.Gimbal.input.pitch;
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

        if (last_auto_active)
        {
            Gimbal.pitch_location_set = -GIMBALMotor_get_data(PITCHMotor).motor_data.para.pos * RAD_TO_DEG;
            Global.Gimbal.input.pitch = Gimbal.pitch_location_set;
        }
        last_auto_active = 0;
        Gimbal.position[0] = -Gimbal.pitch_location_set * DEG_TO_RAD;
        // Gimbal.pitch_speed_set = PID_Cal(&Gimbal.pitch_location_pid, Gimbal.pitch_location_now, Gimbal.pitch_location_set) * DEG_TO_RAD;
        if (Global.Auto.input.Auto_control_online > 0)
            Global.Auto.input.Auto_control_online--;
    }
    else
    {
        // 自瞄
        // static float pitch_ff_filt = 0;
        // pitch_ff_filt += 0.5f * (Global.Auto.input.pitch_ff - pitch_ff_filt);
        static float pitch_ff_filt = 0;
        pitch_ff_filt += 0.5f * (Global.Auto.input.pitch_ff - pitch_ff_filt);

        float vel_ff = PITCH_VEL_FF_GAIN * pitch_ff_filt;
        float acc_ff = PITCH_ACC_FF_GAIN * Global.Auto.input.pitch_acc_ff;
        if (vel_ff > PITCH_VEL_FF_LIMIT)
            vel_ff = PITCH_VEL_FF_LIMIT;
        if (vel_ff < -PITCH_VEL_FF_LIMIT)
            vel_ff = -PITCH_VEL_FF_LIMIT;
        if (acc_ff > PITCH_ACC_FF_LIMIT)
            acc_ff = PITCH_ACC_FF_LIMIT;
        if (acc_ff < -PITCH_ACC_FF_LIMIT)
            acc_ff = -PITCH_ACC_FF_LIMIT;

        float ff_total = vel_ff + acc_ff;
        if (ff_total > PITCH_FF_TOTAL_LIMIT)
            ff_total = PITCH_FF_TOTAL_LIMIT;
        if (ff_total < -PITCH_FF_TOTAL_LIMIT)
            ff_total = -PITCH_FF_TOTAL_LIMIT;

        Gimbal.position[0] = DM_Motor_data[0][1].motor_data.para.pos + (Gimbal.pitch_location_set - Gimbal.pitch_location_now * DEG_TO_RAD) + ff_total;
        // Gimbal.position[0] = Global.Auto.input.shoot_pitch;

        // pitch轴重力补偿
        // Gimbal.force[0] += -(0.45 * cosf(DEG_TO_RAD * Gimbal.pitch_location_now));
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
        GIMBALMotor_set(PITCHMotor, -Gimbal.position[0], 0, 0.0f, 40.0f, 2.0f);
    }
    else
    {
        GIMBALMotor_set(PITCHMotor, 0, 0, 0, 0, 0);
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
            Global.Gimbal.input.pitch = Gimbal.pitch_location_now;
        }
        Gimbal_Calculater();
        // 纠偏阶段强制输出，不受 LOCK 模式影响
        GIMBALMotor_set(PITCHMotor, Gimbal.position[0], 0, 0, 50.0f, 1.5f);
    }
    else
    {
        // 正常控制
        Gimbal_Calculater();
        Gimbal_Controller();
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
