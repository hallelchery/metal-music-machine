#pragma once
#include <cstdint>
#include <cstdlib>

// Simulated stepper motor.
//
// Tracks position in steps and advances toward a target at a capped step rate.
// Stall detection fires when the motor is commanded past its travel limits.
class SimStepper {
public:
    static constexpr int     MAX_STEPS_PER_MS = 2;   // ~2 kHz step rate
    static constexpr int     MIN_POSITION     = 0;
    static constexpr int     MAX_POSITION     = 1000; // arbitrary sim travel range

    SimStepper();

    // Move toward target_pos by at most MAX_STEPS_PER_MS * elapsed_ms steps.
    // Returns true if the target was reached this tick.
    // Sets stall flag if target is outside travel limits.
    bool tick(uint32_t nowMs);

    void setTarget(int target_pos);
    int  getPosition()  const;
    bool isAtTarget()   const;
    bool isStalled()    const;
    void clearStall();
    void deenergize();   // Simulate power cut (SLEEP entry)
    void energize();     // Restore after SLEEP

private:
    int      _position;
    int      _target;
    bool     _stalled;
    bool     _energized;
    uint32_t _lastTickMs;
};


// Simulated servo motor.
//
// Slews toward a target angle at a fixed rate. Faults if commanded outside
// its mechanical travel range.
class SimServo {
public:
    static constexpr float MIN_ANGLE_DEG    = 0.0f;
    static constexpr float MAX_ANGLE_DEG    = 180.0f;
    static constexpr float DEG_PER_MS       = 0.2f;  // ~60° per 300ms

    SimServo();

    // Advance angle toward target by DEG_PER_MS * elapsed_ms.
    // Returns true if target was reached this tick.
    bool tick(uint32_t nowMs);

    void  setTarget(float angle_deg);
    float getAngle()      const;
    bool  isAtTarget()    const;
    bool  isOverRange()   const;

private:
    float    _angle;
    float    _target;
    bool     _overRange;
    uint32_t _lastTickMs;
};