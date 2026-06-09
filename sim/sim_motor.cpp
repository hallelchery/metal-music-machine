#include "sim_motor.h"
#include <cmath>
#include <algorithm>

// =============================================================================
// SimStepper
// =============================================================================

SimStepper::SimStepper()
    : _position(0)
    , _target(0)
    , _stalled(false)
    , _energized(true)
    , _lastTickMs(0)
{}

bool SimStepper::tick(uint32_t nowMs) {
    if (!_energized || _stalled) return false;
    if (_position == _target)    return true;

    uint32_t elapsed = nowMs - _lastTickMs;
    _lastTickMs = nowMs;

    int maxStep = static_cast<int>(elapsed) * MAX_STEPS_PER_MS;
    int delta   = _target - _position;
    int step    = std::clamp(delta, -maxStep, maxStep);

    _position += step;

    // Clamp to travel limits and flag a stall if the target demanded we go beyond them.
    if (_position < MIN_POSITION || _position > MAX_POSITION) {
        _position = std::clamp(_position, MIN_POSITION, MAX_POSITION);
        _stalled  = true;
        return false;
    }

    return (_position == _target);
}

void SimStepper::setTarget(int target_pos) {
    if (target_pos < MIN_POSITION || target_pos > MAX_POSITION) {
        _stalled = true;
        return;
    }
    _target  = target_pos;
    _stalled = false;
}

int  SimStepper::getPosition() const { return _position; }
bool SimStepper::isAtTarget()  const { return _position == _target; }
bool SimStepper::isStalled()   const { return _stalled; }
void SimStepper::clearStall()        { _stalled = false; }

void SimStepper::deenergize() { _energized = false; }
void SimStepper::energize()   { _energized = true; }


// =============================================================================
// SimServo
// =============================================================================

SimServo::SimServo()
    : _angle(90.0f)
    , _target(90.0f)
    , _overRange(false)
    , _lastTickMs(0)
{}

bool SimServo::tick(uint32_t nowMs) {
    if (_overRange) return false;

    uint32_t elapsed = nowMs - _lastTickMs;
    _lastTickMs = nowMs;

    float maxDelta = static_cast<float>(elapsed) * DEG_PER_MS;
    float delta    = _target - _angle;

    if (std::abs(delta) <= maxDelta) {
        _angle = _target;
        return true;
    }

    _angle += (delta > 0.0f) ? maxDelta : -maxDelta;
    return false;
}

void SimServo::setTarget(float angle_deg) {
    if (angle_deg < MIN_ANGLE_DEG || angle_deg > MAX_ANGLE_DEG) {
        _overRange = true;
        return;
    }
    _target    = angle_deg;
    _overRange = false;
}

float SimServo::getAngle()    const { return _angle; }
bool  SimServo::isAtTarget()  const { return _angle == _target; }
bool  SimServo::isOverRange() const { return _overRange; }