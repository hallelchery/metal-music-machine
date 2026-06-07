#pragma once
#include <cstdint>

// Hardware Abstraction Layer (HAL)
// This is the ONLY place the rest of the code is allowed to talk to hardware.
// The sim and Arduino each provide their own implementation of these functions.
// Everything in src/ calls these — and never knows which hardware it's running on.

// --- Time ---
// Returns milliseconds elapsed since program start.
// Mirrors Arduino's millis() so firmware logic is identical on both targets.
uint32_t hal_millis();

// --- Display ---
// Prints a message to whatever "display" exists:
// on sim → terminal (printf), on Arduino → LCD or OLED.
void hal_displayPrint(const char* msg);

// --- Telemetry ---
// Opens the CSV log file. Call once at startup.
void hal_telemetryInit();

// Appends one row to telemetry.csv.
// Parameters are what happened this loop pass.
void hal_telemetryLog(uint32_t timestamp_ms,
                      const char* state_name,
                      const char* event_name);

// --- Motor stubs (placeholders for now) ---
// These do nothing in Iteration 0 — filled in during Iteration 2.
void hal_stepperMove(int motor_id, int steps);
void hal_servoWrite(int servo_id, int angle_deg);