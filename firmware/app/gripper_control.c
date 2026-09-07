#include "gripper_control.h"

#include <string.h>

#include "encos_motor.h"
#include "safety.h"

static GripperConfig g_config;
static GripperStatus g_status;
static uint32_t g_last_communication_ms;
static float g_target_width_mm;
static float g_target_velocity_mm_s;
static float g_target_force_n;

static bool value_in_range(float value, float min_value, float max_value)
{
    return value >= min_value && value <= max_value;
}

void Gripper_Init(const GripperConfig *config)
{
    memset(&g_status, 0, sizeof(g_status));
    memset(&g_config, 0, sizeof(g_config));

    if (config != 0) {
        g_config = *config;
    }

    g_target_width_mm = g_config.width_max_mm;
    g_target_velocity_mm_s = g_config.velocity_max_mm_s;
    g_target_force_n = g_config.force_max_n;
    g_status.state = GRIPPER_STATE_DISABLED;

    Safety_Init(g_config.communication_timeout_ms);
    EncosMotor_Init();
}

void Gripper_Update(uint32_t now_ms)
{
    EncosMotorStatus motor_status;

    EncosMotor_Update();
    EncosMotor_GetStatus(&motor_status);

    g_status.motor_position = motor_status.position;
    g_status.motor_velocity = motor_status.velocity;
    g_status.motor_torque_nm = motor_status.torque_nm;
    g_status.motor_temperature_c = motor_status.temperature_c;

    /* Mechanical mapping motor position -> jaw width will be calibrated later. */

    if (!Safety_CommunicationAlive(now_ms, g_last_communication_ms)) {
        EncosMotor_Stop();
        if (g_status.state != GRIPPER_STATE_DISABLED) {
            g_status.state = GRIPPER_STATE_STOPPED;
        }
    }

    if (motor_status.fault) {
        g_status.error_code = motor_status.error_code;
        g_status.state = GRIPPER_STATE_FAULT;
        EncosMotor_Stop();
    }
}

bool Gripper_Enable(void)
{
    if (!EncosMotor_ProtocolVerified()) {
        g_status.error_code = 1U;
        g_status.state = GRIPPER_STATE_FAULT;
        return false;
    }

    if (!EncosMotor_Enable()) {
        return false;
    }

    g_status.error_code = 0U;
    g_status.state = GRIPPER_STATE_READY;
    return true;
}

void Gripper_Disable(void)
{
    EncosMotor_Disable();
    g_status.state = GRIPPER_STATE_DISABLED;
}

void Gripper_Stop(void)
{
    EncosMotor_Stop();
    if (g_status.state != GRIPPER_STATE_DISABLED) {
        g_status.state = GRIPPER_STATE_STOPPED;
    }
}

bool Gripper_Open(void)
{
    return Gripper_SetWidth(g_config.width_max_mm);
}

bool Gripper_Close(void)
{
    return Gripper_SetWidth(g_config.width_min_mm);
}

bool Gripper_SetWidth(float width_mm)
{
    if (!value_in_range(width_mm, g_config.width_min_mm, g_config.width_max_mm)) {
        g_status.error_code = 2U;
        return false;
    }

    g_target_width_mm = width_mm;

    /* TODO: calibrated kinematics: width_mm -> motor position. */
    /* Until that mapping and CAN protocol are verified, no motion is sent. */
    (void)g_target_width_mm;
    return false;
}

bool Gripper_SetVelocity(float velocity_mm_s)
{
    if (velocity_mm_s < 0.0f || velocity_mm_s > g_config.velocity_max_mm_s) {
        return false;
    }
    g_target_velocity_mm_s = velocity_mm_s;
    return true;
}

bool Gripper_SetForce(float force_n)
{
    if (force_n < 0.0f || force_n > g_config.force_max_n) {
        return false;
    }
    g_target_force_n = force_n;
    return true;
}

void Gripper_OnCommunicationActivity(uint32_t now_ms)
{
    g_last_communication_ms = now_ms;
}

const GripperStatus *Gripper_GetStatus(void)
{
    return &g_status;
}
