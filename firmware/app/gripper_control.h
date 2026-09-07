#ifndef GRIPPER_CONTROL_H
#define GRIPPER_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    GRIPPER_STATE_DISABLED = 0,
    GRIPPER_STATE_READY,
    GRIPPER_STATE_MOVING,
    GRIPPER_STATE_STOPPED,
    GRIPPER_STATE_FAULT
} GripperState;

typedef struct {
    float width_min_mm;
    float width_max_mm;
    float velocity_max_mm_s;
    float force_max_n;
    float kp;
    float ki;
    float kd;
    uint32_t communication_timeout_ms;
} GripperConfig;

typedef struct {
    float width_mm;
    float motor_position;
    float motor_velocity;
    float motor_torque_nm;
    float motor_temperature_c;
    GripperState state;
    uint32_t error_code;
} GripperStatus;

void Gripper_Init(const GripperConfig *config);
void Gripper_Update(uint32_t now_ms);

bool Gripper_Enable(void);
void Gripper_Disable(void);
void Gripper_Stop(void);

bool Gripper_Open(void);
bool Gripper_Close(void);
bool Gripper_SetWidth(float width_mm);
bool Gripper_SetVelocity(float velocity_mm_s);
bool Gripper_SetForce(float force_n);

void Gripper_OnCommunicationActivity(uint32_t now_ms);
const GripperStatus *Gripper_GetStatus(void);

#endif
