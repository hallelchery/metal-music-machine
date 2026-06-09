#pragma once
#include <cstdint>
#include <cstdlib>

// Motor IDs — indices into the controller's internal arrays.
// String count drives the fretting motor count; the instrument has 3 strings.
static constexpr uint8_t STEPPER_COUNT = 3;
static constexpr uint8_t SERVO_COUNT   = 3; // one pluck servo per string

// Homing target: position 0 is the reference (limit-switch) position.
static constexpr int HOMING_TARGET   = 0;

// Tuning target: representative calibration position for all fret steppers.
static constexpr int TUNING_TARGET   = 50;

// MotorController manages all motor motion for the Metal Music Machine.
//
// It is the sole firmware component that calls hal_stepperMove / hal_servoWrite.
// The FSM calls tick() every loop pass and reads status flags to decide when
// to fire EVT_DONE or EVT_FAULT.
//
// Design contract:
//   - tick() is non-blocking; it advances motors by one time-slice per call.
//   - The FSM never waits in a loop — it calls tick() once per loop pass.
//   - Fault and completion are communicated via status flags, not return values.
class MotorController {
public:
    MotorController();

    // Advance all motors by one time-slice. Call every loop pass.
    void tick(uint32_t nowMs);

    // --- Commands ---
    // Each command is non-blocking: it sets targets and returns immediately.
    // The FSM polls isHomingComplete() / isTuningComplete() on subsequent ticks.

    void startHoming();
    void startTuning();

    void setStepperTarget(uint8_t motor_id, int target_pos);
    void setServoTarget(uint8_t servo_id, float angle_deg);

    void deenergizeAll();  // Called on SLEEP entry
    void energizeAll();    // Called on SLEEP exit

    // --- Status queries ---
    bool isHomingComplete();   // Returns true exactly once when all steppers reach HOMING_TARGET.
    bool isTuningComplete();   // Returns true exactly once when all steppers reach TUNING_TARGET.
    bool isFaulted()         const;

    // --- Telemetry accessors ---
    int   getStepperPosition(uint8_t motor_id) const;
    float getServoAngle(uint8_t servo_id)      const;

private:
    // Motor state is owned here; the HAL calls are the only output channel.
    // Position tracking mirrors what the HAL reports back so telemetry is accurate.
    int   _stepperPos[STEPPER_COUNT];
    float _servoAngle[SERVO_COUNT];
    int   _stepperTarget[STEPPER_COUNT];
    float _servoTarget[SERVO_COUNT];

    bool _homingActive;
    bool _tuningActive;
    bool _faulted;

    uint32_t _lastTickMs;

    void _tickStepper(uint8_t id, uint32_t elapsed);
    void _tickServo(uint8_t id, uint32_t elapsed);
};