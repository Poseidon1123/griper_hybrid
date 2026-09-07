#ifndef ENCOS_MOTOR_H
#define ENCOS_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float position;
    float velocity;
    float torque_nm;
    float temperature_c;
    bool fault;
    uint32_t error_code;
} EncosMotorStatus;

void EncosMotor_Init(void);
void EncosMotor_Update(void);

bool EncosMotor_ProtocolVerified(void);
bool EncosMotor_Enable(void);
void EncosMotor_Disable(void);
void EncosMotor_Stop(void);

bool EncosMotor_SetPosition(float position, float velocity_limit, float torque_limit_nm);
void EncosMotor_GetStatus(EncosMotorStatus *status);

/*
 * Hardware integration hooks to implement after STM32CubeMX generation and
 * verification of the official EC-A4310-P2-36 CAN protocol.
 */
void EncosMotor_OnCanFrame(uint32_t can_id, const uint8_t *data, uint8_t dlc);

#endif
