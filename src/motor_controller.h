#pragma once
#include <cstdint>

static constexpr uint8_t NUM_STRINGS    = 3;

// Per-string flat motor ID derivation (see hardware architecture doc):
//   tuning_servo_id  = string_id          (routed via demux)
//   fret_stepper_id  = string_id
//   fret_servo_id    = string_id + 3
//   pluck_servo_id   = string_id + 6
static constexpr uint8_t STEPPER_COUNT  = NUM_STRINGS;       // one fret stepper per string
static constexpr uint8_t SERVO_COUNT    = NUM_STRINGS * 3;   // tuning + fret + pluck per string

static constexpr int   HOMING_TARGET    = 0;
static constexpr int   TUNING_FRET_HOME = 0;   // fret steppers park at 0 during tuning

// Fret position table: maps note index [0..11] to stepper step position.
// Placeholder values — real calibration data replaces these at hardware bring-up.
static constexpr int FRET_POSITIONS[12] = {
     0, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550
};

// Servo angle for pressing a string (fret servo down / up).
static constexpr float FRET_PRESS_ANGLE  = 45.0f;
static constexpr float FRET_LIFT_ANGLE   = 135.0f;

// Pluck servo sweep angles.
static constexpr float PLUCK_STRIKE_ANGLE = 30.0f;
static constexpr float PLUCK_REST_ANGLE   = 90.0f;

// MotorController: the sole component that issues hal_stepperMove / hal_servoWrite.
// The FSM interacts with motors entirely through this class.
//
// Non-blocking contract: every command sets targets and returns immediately.
// tick() advances all motors by one time-slice per call.
class MotorController {
public:
    MotorController();

    void tick(uint32_t nowMs);

    // --- Low-level commands (used by homing) ---
    void startHoming();
    void setStepperTarget(uint8_t motor_id, int target_pos);
    void setServoTarget(uint8_t servo_id, float angle_deg);

    void deenergizeAll();
    void energizeAll();

    // --- Per-string musical commands (used by TUNING, PERFORMING, PRE_COMPOSING) ---
    // commandFret: move fret stepper to the position for note_index, then press fret servo.
    void commandFret(int string_id, int note_index);

    // commandFretLift: lift the fret servo (release string).
    void commandFretLift(int string_id);

    // commandPluck: fire the pluck servo sweep for the given string.
    void commandPluck(int string_id);

    // commandTuningServo: set tuning servo angle for the given string.
    // Caller must have called hal_selectTuningString(string_id) first.
    void commandTuningServo(int string_id, float angle_deg);

    // --- Status queries ---
    bool isHomingComplete();
    bool isFretAtTarget(int string_id)        const;
    bool isPluckComplete(int string_id)       const;
    bool isTuningServoAtTarget(int string_id) const;
    bool isFaulted()                          const;
    void clearFault();

    // --- Telemetry accessors ---
    int   getStepperPosition(uint8_t motor_id) const;
    float getServoAngle(uint8_t servo_id)      const;

private:
    int   _stepperPos[STEPPER_COUNT];
    int   _stepperTarget[STEPPER_COUNT];
    float _servoAngle[SERVO_COUNT];
    float _servoTarget[SERVO_COUNT];

    bool _homingActive;
    bool _faulted;

    uint32_t _lastTickMs;

    void _tickStepper(uint8_t id, uint32_t elapsed);
    void _tickServo(uint8_t id, uint32_t elapsed);

    // ID derivation helpers — keep the indexing convention in one place.
    uint8_t _tuningServoId(int s)  const { return static_cast<uint8_t>(s); }
    uint8_t _fretStepperId(int s)  const { return static_cast<uint8_t>(s); }
    uint8_t _fretServoId(int s)    const { return static_cast<uint8_t>(s + 3); }
    uint8_t _pluckServoId(int s)   const { return static_cast<uint8_t>(s + 6); }
};