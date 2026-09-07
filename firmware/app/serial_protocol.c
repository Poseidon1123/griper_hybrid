#include "serial_protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gripper_control.h"

static SerialProtocolTxFn g_tx_fn = 0;

static void tx(const char *text)
{
    if (g_tx_fn != 0 && text != 0) {
        g_tx_fn(text);
    }
}

static const char *state_to_text(GripperState state)
{
    switch (state) {
    case GRIPPER_STATE_DISABLED: return "DISABLED";
    case GRIPPER_STATE_READY: return "READY";
    case GRIPPER_STATE_MOVING: return "MOVING";
    case GRIPPER_STATE_STOPPED: return "STOPPED";
    case GRIPPER_STATE_FAULT: return "FAULT";
    default: return "UNKNOWN";
    }
}

static int parse_float_after_prefix(const char *line, const char *prefix, float *value)
{
    size_t prefix_len = strlen(prefix);
    char *end_ptr = 0;
    float parsed;

    if (strncmp(line, prefix, prefix_len) != 0) {
        return 0;
    }

    parsed = strtof(line + prefix_len, &end_ptr);
    if (end_ptr == line + prefix_len || *end_ptr != '\0') {
        return -1;
    }

    *value = parsed;
    return 1;
}

void SerialProtocol_Init(SerialProtocolTxFn tx_fn)
{
    g_tx_fn = tx_fn;
}

void SerialProtocol_ProcessLine(const char *line, uint32_t now_ms)
{
    float value = 0.0f;
    int parse_result;

    if (line == 0 || *line == '\0') {
        return;
    }

    Gripper_OnCommunicationActivity(now_ms);

    if (strcmp(line, "PING") == 0) {
        tx("PONG\n");
        return;
    }

    if (strcmp(line, "ENABLE") == 0) {
        tx(Gripper_Enable() ? "OK,ENABLE\n" : "ERROR,ENABLE_FAILED\n");
        return;
    }

    if (strcmp(line, "DISABLE") == 0) {
        Gripper_Disable();
        tx("OK,DISABLE\n");
        return;
    }

    if (strcmp(line, "STOP") == 0) {
        Gripper_Stop();
        tx("OK,STOP\n");
        return;
    }

    if (strcmp(line, "OPEN") == 0) {
        tx(Gripper_Open() ? "OK,OPEN\n" : "ERROR,OPEN_FAILED\n");
        return;
    }

    if (strcmp(line, "CLOSE") == 0) {
        tx(Gripper_Close() ? "OK,CLOSE\n" : "ERROR,CLOSE_FAILED\n");
        return;
    }

    if (strcmp(line, "GET_STATUS") == 0) {
        SerialProtocol_SendStatus();
        return;
    }

    parse_result = parse_float_after_prefix(line, "SET_WIDTH ", &value);
    if (parse_result != 0) {
        tx(parse_result > 0 && Gripper_SetWidth(value)
               ? "OK,SET_WIDTH\n"
               : "ERROR,SET_WIDTH_FAILED\n");
        return;
    }

    parse_result = parse_float_after_prefix(line, "SET_VELOCITY ", &value);
    if (parse_result != 0) {
        tx(parse_result > 0 && Gripper_SetVelocity(value)
               ? "OK,SET_VELOCITY\n"
               : "ERROR,SET_VELOCITY_FAILED\n");
        return;
    }

    parse_result = parse_float_after_prefix(line, "SET_FORCE ", &value);
    if (parse_result != 0) {
        tx(parse_result > 0 && Gripper_SetForce(value)
               ? "OK,SET_FORCE\n"
               : "ERROR,SET_FORCE_FAILED\n");
        return;
    }

    tx("ERROR,UNKNOWN_COMMAND\n");
}

void SerialProtocol_SendStatus(void)
{
    const GripperStatus *status = Gripper_GetStatus();
    char buffer[96];

    if (status == 0) {
        return;
    }

    snprintf(buffer, sizeof(buffer), "WIDTH,%.2f\n", status->width_mm);
    tx(buffer);
    snprintf(buffer, sizeof(buffer), "POS,%.4f\n", status->motor_position);
    tx(buffer);
    snprintf(buffer, sizeof(buffer), "VEL,%.4f\n", status->motor_velocity);
    tx(buffer);
    snprintf(buffer, sizeof(buffer), "TORQUE,%.3f\n", status->motor_torque_nm);
    tx(buffer);
    snprintf(buffer, sizeof(buffer), "TEMP,%.1f\n", status->motor_temperature_c);
    tx(buffer);
    snprintf(buffer, sizeof(buffer), "STATE,%s\n", state_to_text(status->state));
    tx(buffer);
}
