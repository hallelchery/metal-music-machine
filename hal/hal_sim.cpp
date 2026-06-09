#include "hal.h"
#include <cstdio>
#include <cstdint>
#include <chrono>

// We need a reference point: the moment the program started.
// Everything in chrono is scoped under std::chrono — C++'s time library.
static auto program_start = std::chrono::steady_clock::now();

// File pointer for our telemetry CSV.
// 'static' here means this variable is only visible inside this file.
static FILE* telemetry_file = nullptr;

// --- Time ---
uint32_t hal_millis() {
    auto now = std::chrono::steady_clock::now();
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
        // Added queue_depth column that shows buffer pressure during stress tests.
        fprintf(telemetry_file, "timestamp_ms,state,event,queue_depth\n");
    }
}

void hal_telemetryLog(uint32_t timestamp_ms,
                      const char* state_name,
                      const char* event_name,
                      uint8_t queue_depth) {
    if (telemetry_file) {
        fprintf(telemetry_file, "%u,%s,%s,%u\n",
                timestamp_ms, state_name, event_name, queue_depth);
        fflush(telemetry_file);
    }
}

// --- Motor stubs ---
void hal_stepperMove(int motor_id, int steps) {
    // Stub: does nothing in Iteration 0
    (void)motor_id; (void)steps;
}

void hal_servoWrite(int servo_id, int angle_deg) {
    // Stub: does nothing in Iteration 0
    (void)servo_id; (void)angle_deg;
}