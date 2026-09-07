#ifndef SAFETY_H
#define SAFETY_H

#include <stdbool.h>
#include <stdint.h>

void Safety_Init(uint32_t communication_timeout_ms);
bool Safety_CommunicationAlive(uint32_t now_ms, uint32_t last_activity_ms);
bool Safety_WidthValid(float width_mm, float min_mm, float max_mm);
bool Safety_VelocityValid(float velocity_mm_s, float max_mm_s);
bool Safety_ForceValid(float force_n, float max_force_n);

#endif
