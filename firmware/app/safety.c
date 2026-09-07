#include "safety.h"

static uint32_t g_communication_timeout_ms = 200U;

void Safety_Init(uint32_t communication_timeout_ms)
{
    if (communication_timeout_ms > 0U) {
        g_communication_timeout_ms = communication_timeout_ms;
    }
}

bool Safety_CommunicationAlive(uint32_t now_ms, uint32_t last_activity_ms)
{
    return (uint32_t)(now_ms - last_activity_ms) <= g_communication_timeout_ms;
}

bool Safety_WidthValid(float width_mm, float min_mm, float max_mm)
{
    return width_mm >= min_mm && width_mm <= max_mm;
}

bool Safety_VelocityValid(float velocity_mm_s, float max_mm_s)
{
    return velocity_mm_s >= 0.0f && velocity_mm_s <= max_mm_s;
}

bool Safety_ForceValid(float force_n, float max_force_n)
{
    return force_n >= 0.0f && force_n <= max_force_n;
}
