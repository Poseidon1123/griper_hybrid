#include "encos_motor.h"

#include <string.h>

static EncosMotorStatus g_status;

void EncosMotor_Init(void)
{
    memset(&g_status, 0, sizeof(g_status));
}

void EncosMotor_Update(void)
{
    /* TODO: process fresh CAN feedback and communication timeout. */
}

bool EncosMotor_ProtocolVerified(void)
{
    /* Deliberately false until the exact EC-A4310-P2-36 protocol is confirmed. */
    return false;
}

bool EncosMotor_Enable(void)
{
    if (!EncosMotor_ProtocolVerified()) {
        return false;
    }

    /* TODO: send verified motor-enable CAN frame. */
    return false;
}

void EncosMotor_Disable(void)
{
    /* TODO: send verified motor-disable CAN frame. */
}

void EncosMotor_Stop(void)
{
    /* TODO: send verified stop/zero-torque command according to motor protocol. */
}

bool EncosMotor_SetPosition(float position, float velocity_limit, float torque_limit_nm)
{
    (void)position;
    (void)velocity_limit;
    (void)torque_limit_nm;

    if (!EncosMotor_ProtocolVerified()) {
        return false;
    }

    /* TODO: encode and transmit the verified position-control frame. */
    return false;
}

void EncosMotor_GetStatus(EncosMotorStatus *status)
{
    if (status != 0) {
        *status = g_status;
    }
}

void EncosMotor_OnCanFrame(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    (void)can_id;
    (void)data;
    (void)dlc;

    /* TODO: decode only after CAN feedback IDs and scaling are verified. */
}
