#pragma once
#include <cstdint>

static constexpr uint8_t NUM_STRINGS    = 3;

// Per-string flat motor ID derivation:
//   tuning_servo_id  = string_id          (routed via demux)
//   fret_stepper_id  = string_id
//   fret_servo_id    = string_id + 3
//   pluck_servo_id   = string_id + 6
//   damper_servo_id  = string_id + 9
static constexpr uint8_t STEPPER_COUNT  = NUM_STRINGS;
static constexpr uint8_t SERVO_COUNT    = NUM_STRINGS * 4;   // tuning + fret + pluck + damper

static constexpr int   HOMING_TARGET    = 0;
static constexpr int   TUNING_FRET_HOME = 0;

static constexpr int FRET_POSITIONS[12] = {
     0, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550
};

static constexpr float FRET_PRESS_ANGLE  = 45.0f;
static constexpr float FRET_LIFT_ANGLE   = 135.0f;

// Pluck servo angles. During operation the servo toggles between these two
// angles — either direction produces a valid pluck stroke. The "up" position
// is also the park position used during clean shutdown.
static constexpr float PLUCK_UP_ANGLE   = 90.0f;
static constexpr float PLUCK_DOWN_ANGLE = 30.0f;

// Damper servo angles.
static constexpr float DAMPER_ENGAGE_ANGLE  = 45.0f;   // damper touches string
static constexpr float DAMPER_LIFT_ANGLE    = 135.0f;  // damper clear of string

class MotorController {
public:
    MotorController();

    void tick(uint32_t nowMs);

    // --- Low-level commands ---
    void startHoming();
    void setStepperTarget(uint8_t motor_id, int target_pos);
    void setServoTarget(uint8_t servo_id, float angle_deg);

    void deenergizeAll();
    void energizeAll();

    // --- Per-string musical commands ---
    void commandFret(int string_id, int note_index);
    void commandFretLift(int string_id);

    // Toggle pluck: alternates between PLUCK_DOWN_ANGLE and PLUCK_UP_ANGLE each call.
    // Either direction produces a pluck stroke; no operational distinction is made.
    void commandPluck(int string_id);

    void commandTuningServo(int string_id, float angle_deg);

    void commandDamperEngage(int string_id);
    void commandDamperLift(int string_id);

    // Park all actuators to their clean resting positions:
    // pluck → up, fret servo → lifted, damper → lifted, fret stepper → home.
    // Called on clean shutdown and SLEEP entry.
    void parkAll();

    // --- Status queries ---
    bool isHomingComplete();
    bool isFretAtTarget(int string_id)        const;
    bool isPluckComplete(int string_id)       const;
    bool isTuningServoAtTarget(int string_id) const;
    bool isDamperAtTarget(int string_id)      const;
    bool isParkComplete()                     const;
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

    bool     _homingActive;
    bool     _faulted;
    bool     _pluckState[NUM_STRINGS];   // false = currently at UP, true = currently at DOWN
    uint32_t _lastTickMs;

    void _tickStepper(uint8_t id, uint32_t elapsed);
    void _tickServo(uint8_t id, uint32_t elapsed);

    uint8_t _tuningServoId(int s)  const { return static_cast<uint8_t>(s); }
    uint8_t _fretStepperId(int s)  const { return static_cast<uint8_t>(s); }
    uint8_t _fretServoId(int s)    const { return static_cast<uint8_t>(s + 3); }
    uint8_t _pluckServoId(int s)   const { return static_cast<uint8_t>(s + 6); }
    uint8_t _damperServoId(int s)  const { return static_cast<uint8_t>(s + 9); }
};