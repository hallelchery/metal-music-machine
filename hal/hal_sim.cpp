#include "hal.h"
#include <cstdio>
#include <cstdint>
#include <chrono>

static auto   program_start  = std::chrono::steady_clock::now();
static FILE*  telemetry_file = nullptr;

// --- Time ---

uint32_t hal_millis() {
    auto now     = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - program_start);
    return static_cast<uint32_t>(elapsed.count());
}

// --- Display ---

void hal_displayPrint(const char* msg) {
    printf("[DISPLAY] %s\n", msg);
}

// --- Telemetry ---

void hal_telemetryInit() {
    telemetry_file = fopen("telemetry.csv", "w");
    if (telemetry_file) {
        fprintf(telemetry_file,
                "timestamp_ms,state,event,queue_depth\n");
    }
}

void hal_telemetryLog(uint32_t    timestamp_ms,
                      const char* state_name,
                      const char* event_name,
                      uint8_t     queue_depth) {
    if (!telemetry_file) return;
    fprintf(telemetry_file, "%u,%s,%s,%u\n",
            timestamp_ms, state_name, event_name, queue_depth);
    fflush(telemetry_file);
}

void hal_telemetryLogMotors(uint32_t    timestamp_ms,
                            const char* state_name,
                            int         s0, int s1, int s2,
                            float       servo0) {
    if (!telemetry_file) return;
    // Motor telemetry rows are distinguished by the "MOTORS" event label.
    fprintf(telemetry_file, "%u,%s,MOTORS,0,%d,%d,%d,%.1f\n",
            timestamp_ms, state_name, s0, s1, s2, servo0);
    fflush(telemetry_file);
}

// --- Motors ---
// In the sim, these are no-ops — SimStepper/SimServo in sim_motor.cpp hold
// the authoritative position. On real hardware these would write to driver ICs.

void hal_stepperMove(int motor_id, int steps) {
    (void)motor_id; (void)steps;
}

void hal_servoWrite(int servo_id, int angle_deg) {
    (void)servo_id; (void)angle_deg;
}