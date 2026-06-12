#include "hal.h"
#include <cstdio>
#include <cstdint>
#include <chrono>
#include <cmath>

static auto  program_start  = std::chrono::steady_clock::now();
static FILE* telemetry_file = nullptr;

// Physics model: maps tuning servo angle to simulated string frequency.
// Each string has a different resting tension, so the same angle produces
// different pitch across strings. The relationship is linear in this model;
// real strings follow a more complex curve but this is sufficient for sim.
static constexpr float BASE_HZ[3]       = { 196.0f, 246.9f, 293.7f }; // G3, B3, D4
static constexpr float HZ_PER_DEGREE[3] = {   1.2f,   1.4f,   1.6f };

static int   active_tuning_string = 0;
static float tuning_servo_angle[3] = { 90.0f, 90.0f, 90.0f };

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
                "timestamp_ms,state,event,queue_depth,"
                "s0_pos,s1_pos,s2_pos,servo0_angle,"
                "tune_string,tune_attempt,measured_hz,target_hz,tune_servo_angle,"
                "note_index,note_string\n");
    }
}

void hal_telemetryLog(uint32_t    timestamp_ms,
                      const char* state_name,
                      const char* event_name,
                      uint8_t     queue_depth) {
    if (!telemetry_file) return;
    fprintf(telemetry_file, "%u,%s,%s,%u,,,,,,,,,,\n",
            timestamp_ms, state_name, event_name, queue_depth);
    fflush(telemetry_file);
}

void hal_telemetryLogMotors(uint32_t    timestamp_ms,
                            const char* state_name,
                            int s0, int s1, int s2,
                            float servo0) {
    if (!telemetry_file) return;
    fprintf(telemetry_file, "%u,%s,MOTORS,0,%d,%d,%d,%.1f,,,,,,,\n",
            timestamp_ms, state_name, s0, s1, s2, servo0);
    fflush(telemetry_file);
}

void hal_telemetryLogTuning(uint32_t    timestamp_ms,
                            int         string_id,
                            int         attempt,
                            float       measured_hz,
                            float       target_hz,
                            float       servo_angle) {
    if (!telemetry_file) return;
    fprintf(telemetry_file, "%u,TUNING,TUNE_SAMPLE,0,,,,,%d,%d,%.2f,%.2f,%.1f,,\n",
            timestamp_ms, string_id, attempt, measured_hz, target_hz, servo_angle);
    fflush(telemetry_file);
}

void hal_telemetryLogNote(uint32_t timestamp_ms,
                          int      note_index,
                          int      string_id) {
    if (!telemetry_file) return;
    fprintf(telemetry_file, "%u,PRE_COMPOSING,NOTE_SELECT,0,,,,,,,,,,,%d,%d\n",
            timestamp_ms, note_index, string_id);
    fflush(telemetry_file);
}

// --- Motors ---

void hal_stepperMove(int motor_id, int steps) {
    (void)motor_id; (void)steps;
}

void hal_servoWrite(int servo_id, int angle_deg) {
    // Track tuning servo angles for the pitch model.
    // Tuning servo IDs are 0, 1, 2 (one per string, per motor indexing convention).
    if (servo_id >= 0 && servo_id < 3) {
        tuning_servo_angle[servo_id] = static_cast<float>(angle_deg);
    }
    (void)servo_id;
}

// --- Pitch Sensing ---

float hal_measurePitch(int string_id) {
    if (string_id < 0 || string_id > 2) return 0.0f;
    float angle = tuning_servo_angle[string_id];
    // Add a small noise term so the sim doesn't converge perfectly in one step,
    // forcing the closed-loop controller to actually iterate.
    float noise = ((float)(rand() % 100) / 100.0f - 0.5f) * 2.0f; // ±1 Hz
    return BASE_HZ[string_id] + HZ_PER_DEGREE[string_id] * (angle - 90.0f) + noise;
}

// --- Demultiplexer ---

void hal_selectTuningString(int string_id) {
    active_tuning_string = string_id;
    printf("[DEMUX] Active tuning string: %d\n", string_id);
}