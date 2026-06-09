#pragma once
#include <cstdint>

// Hardware Abstraction Layer (HAL)
//
// The sole interface between portable firmware (src/) and hardware.
// Both hal_sim.cpp and hal_arduino.cpp implement every function declared here.
// Nothing in src/ may include platform-specific headers.

// --- Time ---
// Milliseconds elapsed since program start. Mirrors Arduino's millis().
uint32_t hal_millis();

// --- Display ---
// Renders a message on the output device (terminal in sim, LCD on hardware).
void hal_displayPrint(const char* msg);

// --- Telemetry ---
void hal_telemetryInit();

// Appends one row to telemetry.csv.
void hal_telemetryLog(uint32_t    timestamp_ms,
                      const char* state_name,
                      const char* event_name,
                      uint8_t     queue_depth);

// Appends one row including motor positions (called each loop pass in motor-active states).
void hal_telemetryLogMotors(uint32_t    timestamp_ms,
                            const char* state_name,
                            int         stepper0_pos,
                            int         stepper1_pos,
                            int         stepper2_pos,
                            float       servo0_angle);

// --- Motors ---
// Step delta: positive = forward, negative = reverse.
void hal_stepperMove(int motor_id, int steps);

// Absolute angle command in degrees [0, 180].
void hal_servoWrite(int servo_id, int angle_deg);