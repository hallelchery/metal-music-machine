#pragma once
#include <cstdint>

// Hardware Abstraction Layer (HAL)
//
// The sole interface between portable firmware (src/) and hardware.
// Both hal_sim.cpp and hal_arduino.cpp implement every function declared here.
// Nothing in src/ may include platform-specific headers.

// --- Time ---
uint32_t hal_millis();

// --- Display ---
void hal_displayPrint(const char* msg);

// --- Telemetry ---
void hal_telemetryInit();

void hal_telemetryLog(uint32_t    timestamp_ms,
                      const char* state_name,
                      const char* event_name,
                      uint8_t     queue_depth);

void hal_telemetryLogMotors(uint32_t    timestamp_ms,
                            const char* state_name,
                            int         stepper0_pos,
                            int         stepper1_pos,
                            int         stepper2_pos,
                            float       servo0_angle);

// Logs one tuning correction attempt for a single string.
void hal_telemetryLogTuning(uint32_t    timestamp_ms,
                            int         string_id,
                            int         attempt,
                            float       measured_hz,
                            float       target_hz,
                            float       servo_angle);

// Logs a note selection event from PRE_COMPOSING.
void hal_telemetryLogNote(uint32_t    timestamp_ms,
                          int         note_index,
                          int         string_id);

// --- Motors ---
void hal_stepperMove(int motor_id, int steps);
void hal_servoWrite(int servo_id, int angle_deg);

// --- Pitch Sensing ---
// In sim: returns a physics-modeled frequency derived from tuning servo angle.
// On Teensy: runs ADC sampling + autocorrelation to extract fundamental frequency.
// string_id: 0, 1, or 2.
float hal_measurePitch(int string_id);

// --- Demultiplexer ---
// Routes the shared tuning PWM signal to the specified string's tuning servo.
// Must be called before any hal_servoWrite targeting a tuning servo.
// In sim: prints the active string. On hardware: sets demux select lines.
void hal_selectTuningString(int string_id);