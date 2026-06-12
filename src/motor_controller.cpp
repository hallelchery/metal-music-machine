#include "motor_controller.h"
#include "../hal/hal.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

static constexpr int   STEPPER_MAX_STEPS_PER_MS = 2;
static constexpr float SERVO_DEG_PER_MS         = 0.2f;
static constexpr int   STEPPER_MIN_POS          = 0;
static constexpr int   STEPPER_MAX_POS          = 1000;
static constexpr float SERVO_MIN_DEG            = 0.0f;
static constexpr float SERVO_MAX_DEG            = 180.0f;
static constexpr float SERVO_EPSILON            = 0.5f;

MotorController::MotorController()
    : _homingActive(false)
    , _faulted(false)
    , _lastTickMs(0)
{
    for (uint8_t i = 0; i < STEPPER_COUNT; i++) {
        _stepperPos[i]    = 0;
        _stepperTarget[i] = 0;
    }
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        _servoAngle[i]  = 90.0f;
        _servoTarget[i] = 90.0f;
    }
}

void MotorController::tick(uint32_t nowMs) {
    if (_faulted) return;
    uint32_t elapsed = nowMs - _lastTickMs;
    _lastTickMs = nowMs;
    for (uint8_t i = 0; i < STEPPER_COUNT; i++) _tickStepper(i, elapsed);
    for (uint8_t i = 0; i < SERVO_COUNT;   i++) _tickServo(i, elapsed);
}

void MotorController::_tickStepper(uint8_t id, uint32_t elapsed) {
    int pos    = _stepperPos[id];
    int target = _stepperTarget[id];
    if (pos == target) return;

    int maxStep = static_cast<int>(elapsed) * STEPPER_MAX_STEPS_PER_MS;
    int delta   = target - pos;
    int step    = std::clamp(delta, -maxStep, maxStep);
    int newPos  = pos + step;

    if (newPos < STEPPER_MIN_POS || newPos > STEPPER_MAX_POS) {
        _faulted = true;
        return;
    }
    _stepperPos[id] = newPos;
    hal_stepperMove(id, step);
}

void MotorController::_tickServo(uint8_t id, uint32_t elapsed) {
    float angle  = _servoAngle[id];
    float target = _servoTarget[id];
    if (std::abs(angle - target) < SERVO_EPSILON) return;

    float maxDelta = static_cast<float>(elapsed) * SERVO_DEG_PER_MS;
    float delta    = target - angle;
    float move     = (std::abs(delta) <= maxDelta) ? delta
                   : (delta > 0.0f ? maxDelta : -maxDelta);
    float newAngle = angle + move;

    if (newAngle < SERVO_MIN_DEG || newAngle > SERVO_MAX_DEG) {
        _faulted = true;
        return;
    }
    _servoAngle[id] = newAngle;
    hal_servoWrite(id, static_cast<int>(newAngle));
}

// --- Low-level commands ---

void MotorController::startHoming() {
    _faulted      = false;
    _homingActive = true;
    for (uint8_t i = 0; i < STEPPER_COUNT; i++) {
        _stepperTarget[i] = HOMING_TARGET;
    }
}

void MotorController::setStepperTarget(uint8_t motor_id, int target_pos) {
    if (motor_id >= STEPPER_COUNT) return;
    if (target_pos < STEPPER_MIN_POS || target_pos > STEPPER_MAX_POS) {
        _faulted = true;
        return;
    }
    _stepperTarget[motor_id] = target_pos;
}

void MotorController::setServoTarget(uint8_t servo_id, float angle_deg) {
    if (servo_id >= SERVO_COUNT) return;
    if (angle_deg < SERVO_MIN_DEG || angle_deg > SERVO_MAX_DEG) {
        _faulted = true;
        return;
    }
    _servoTarget[servo_id] = angle_deg;
}

void MotorController::deenergizeAll() {
    for (uint8_t i = 0; i < STEPPER_COUNT; i++) {
        hal_stepperMove(i, 0);
    }
}

void MotorController::energizeAll() {}

// --- Per-string musical commands ---

void MotorController::commandFret(int string_id, int note_index) {
    if (note_index < 0 || note_index > 11) return;
    int pos = FRET_POSITIONS[note_index];
    setStepperTarget(_fretStepperId(string_id), pos);
    setServoTarget(_fretServoId(string_id), FRET_PRESS_ANGLE);
}

void MotorController::commandFretLift(int string_id) {
    setServoTarget(_fretServoId(string_id), FRET_LIFT_ANGLE);
}

void MotorController::commandPluck(int string_id) {
    setServoTarget(_pluckServoId(string_id), PLUCK_STRIKE_ANGLE);
}

void MotorController::commandTuningServo(int string_id, float angle_deg) {
    setServoTarget(_tuningServoId(string_id), angle_deg);
}

// --- Status queries ---

bool MotorController::isHomingComplete() {
    if (!_homingActive || _faulted) return false;
    for (uint8_t i = 0; i < STEPPER_COUNT; i++) {
        if (_stepperPos[i] != HOMING_TARGET) return false;
    }
    _homingActive = false;
    return true;
}

bool MotorController::isFretAtTarget(int string_id) const {
    uint8_t stepId = _fretStepperId(string_id);
    uint8_t srvId  = _fretServoId(string_id);
    return (_stepperPos[stepId] == _stepperTarget[stepId]) &&
           (std::abs(_servoAngle[srvId] - _servoTarget[srvId]) < SERVO_EPSILON);
}

bool MotorController::isPluckComplete(int string_id) const {
    uint8_t id = _pluckServoId(string_id);
    return std::abs(_servoAngle[id] - PLUCK_STRIKE_ANGLE) < SERVO_EPSILON;
}

bool MotorController::isTuningServoAtTarget(int string_id) const {
    uint8_t id = _tuningServoId(string_id);
    return std::abs(_servoAngle[id] - _servoTarget[id]) < SERVO_EPSILON;
}

bool MotorController::isFaulted() const { return _faulted; }
void MotorController::clearFault()      { _faulted = false; }

int   MotorController::getStepperPosition(uint8_t id) const {
    return (id < STEPPER_COUNT) ? _stepperPos[id] : -1;
}
float MotorController::getServoAngle(uint8_t id) const {
    return (id < SERVO_COUNT) ? _servoAngle[id] : -1.0f;
}