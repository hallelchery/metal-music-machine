#include "fsm.h"
#include "event_detector.h"
#include "../hal/hal.h"
#include <cstdio>
#include <cmath>

static constexpr float TARGET_HZ[3] = { 196.0f, 246.9f, 293.7f };

FSM::FSM(MotorController& controller, EventDetector& detector, Composer& composer)
    : _state(State::HOMING)
    , _homingComplete(false)
    , _motors(controller)
    , _detector(detector)
    , _composer(composer)
    , _tuningSub(TuningSubState::SELECT_STRING)
    , _tuningString(0)
    , _tuningAttempt(0)
    , _tuningPluckCount(0)
    , _tuningSettleStart(0)
    , _tuningPitchSum(0.0f)
    , _tuningReadCount(0)
    , _perfNoteIdx(0)
    , _perfPluckStart(0)
    , _perfGapStart(0)
    , _perfWaitingPluck(false)
    , _perfWaitingGap(false)
    , _perfFretDone(false)
    , _preCompSub(PreCompSubState::MENU_NAVIGATE)
    , _preCompNoteIdx(0)
    , _preCompStringIdx(0)
    , _preCompPluckStart(0)
    , _preCompLastNote(0)
    , _composingDone(false)
    , _parkingForShutdown(false)
    , _parkStartMs(0)
{
    for (int i = 0; i < 3; i++) _tuningTargetHz[i] = TARGET_HZ[i];
}

void FSM::begin() {
    printf("[FSM] Starting up. Entering HOMING.\n");
    onEnter(_state);
}

bool FSM::isParkingDone() const {
    return _parkingForShutdown && _motors.isParkComplete();
}

void FSM::_startPark() {
    _parkingForShutdown = true;
    _parkStartMs        = hal_millis();
    _motors.parkAll();
    printf("[FSM] Parking all actuators for shutdown.\n");
}

void FSM::update(Event evt) {
    during(_state);

    if (evt == Event::EVT_NONE) return;

    State next = computeNextState(_state, evt);

    if (next != _state) {
        onExit(_state);
        _state = next;
        onEnter(_state);
    }
}

// -----------------------------------------------------------------------------
// onEnter
// -----------------------------------------------------------------------------
void FSM::onEnter(State s) {
    printf("[FSM] --> Entering: %s\n", stateName(s));
    hal_displayPrint(stateName(s));

    switch (s) {
        case State::HOMING:
            printf("  [HOMING] Driving motors to reference positions...\n");
            _motors.startHoming();
            break;

        case State::IDLE:
            if (_homingComplete) {
                printf("  [IDLE] Ready. c=Compose  p=Pre-compose  t=Tune  x=Perform\n");
            } else {
                printf("  [IDLE] Homing not complete. TUNE and PERFORM are locked.\n");
            }
            break;

        case State::SLEEP:
            printf("  [SLEEP] Entering low-power standby. Parking motors.\n");
            _motors.parkAll();
            _motors.deenergizeAll();
            break;

        case State::PRE_COMPOSING:
            _preCompSub       = PreCompSubState::MENU_NAVIGATE;
            _preCompNoteIdx   = 0;
            _preCompStringIdx = 0;
            _preCompLastNote  = 0;
            printf("  [PRE_COMPOSING] Note: %d  String: %d  |  UP/DOWN=scroll  N=play  C=compose\n",
                   _preCompNoteIdx, _preCompStringIdx);
            break;

        case State::COMPOSING:
            printf("  [COMPOSING] Starting Markov chain composition (%d notes)...\n", MAX_SEQ_LEN);
            _composingDone = false;
            _composer.beginComposing(0);
            break;

        case State::TUNING:
            printf("  [TUNING] Starting closed-loop pitch calibration...\n");
            _tuningSub    = TuningSubState::SELECT_STRING;
            _tuningString = 0;
            _tuningAttempt= 0;
            for (int s = 0; s < NUM_STRINGS; s++) _motors.commandDamperEngage(s);
            break;

        case State::PERFORMING: {
            int len = _composer.getLength();
            printf("  [PERFORMING] Playing composed sequence (%d notes).\n", len > 0 ? len : 0);
            _perfNoteIdx      = 0;
            _perfWaitingPluck = false;
            _perfWaitingGap   = false;
            _perfFretDone     = false;
            break;
        }

        case State::ERROR_STATE:
            printf("  [ERROR] FAULT DETECTED. Hold S for 2 seconds to reset.\n");
            break;
    }
}

// -----------------------------------------------------------------------------
// during
// -----------------------------------------------------------------------------
void FSM::during(State s) {
    _motors.tick(hal_millis());

    if (_motors.isFaulted() && s != State::ERROR_STATE) {
        _detector.injectFault();
        return;
    }

    switch (s) {
        case State::HOMING:        _duringHoming();        break;
        case State::TUNING:        _duringTuning();        break;
        case State::PERFORMING:    _duringPerforming();    break;
        case State::PRE_COMPOSING: _duringPreComposing();  break;
        case State::COMPOSING:     _duringComposing();     break;
        default: break;
    }
}

// -----------------------------------------------------------------------------
// _duringHoming
// -----------------------------------------------------------------------------
void FSM::_duringHoming() {
    if (_motors.isHomingComplete()) {
        printf("  [HOMING] All steppers at reference. Homing complete.\n");
        _homingComplete = true;
        _detector.injectDone();
    }
}

// -----------------------------------------------------------------------------
// _duringComposing — generates one note per loop pass until buffer is full
// -----------------------------------------------------------------------------
void FSM::_duringComposing() {
    if (_composingDone) return;

    bool finished = _composer.step();
    if (finished) {
        _composingDone = true;
        _composer.logDistribution();
        printf("  [COMPOSING] Sequence complete (%d notes). Firing EVT_DONE.\n",
               _composer.getLength());
        _detector.injectDone();
    }
}

// -----------------------------------------------------------------------------
// _duringTuning — closed-loop pitch correction sub-state machine
// -----------------------------------------------------------------------------
void FSM::_duringTuning() {
    uint32_t now = hal_millis();

    switch (_tuningSub) {

        case TuningSubState::SELECT_STRING:
            if (_tuningString >= NUM_STRINGS) {
                printf("  [TUNING] All strings tuned. Firing EVT_DONE.\n");
                _tuningSub = TuningSubState::DONE;
                _detector.injectDone();
                return;
            }
            hal_selectTuningString(_tuningString);
            _tuningAttempt    = 0;
            _tuningPitchSum   = 0.0f;
            _tuningReadCount  = 0;
            _tuningPluckCount = 0;
            _motors.commandDamperLift(_tuningString);
            printf("  [TUNING] String %d — target %.1f Hz\n",
                   _tuningString, _tuningTargetHz[_tuningString]);
            _tuningSub = TuningSubState::COMMANDING_SERVO;
            break;

        case TuningSubState::COMMANDING_SERVO:
            if (_motors.isTuningServoAtTarget(_tuningString)) {
                _tuningPluckCount = 0;
                _tuningSub = TuningSubState::PLUCKING;
            }
            break;

        case TuningSubState::PLUCKING:
            if (_tuningPluckCount < TUNING_N_PLUCKS) {
                _motors.commandPluck(_tuningString);
                _tuningPluckCount++;
            } else {
                _tuningSettleStart = now;
                _tuningPitchSum    = 0.0f;
                _tuningReadCount   = 0;
                _tuningSub = TuningSubState::LISTENING;
            }
            break;

        case TuningSubState::LISTENING:
            if ((now - _tuningSettleStart) >= TUNING_MIC_SETTLE_MS) {
                float hz = hal_measurePitch(_tuningString);
                _tuningPitchSum  += hz;
                _tuningReadCount += 1;
                _tuningSub = TuningSubState::EVALUATING;
            }
            break;

        case TuningSubState::EVALUATING: {
            float measured = _tuningPitchSum / static_cast<float>(_tuningReadCount);
            float target   = _tuningTargetHz[_tuningString];
            float error    = measured - target;

            hal_telemetryLogTuning(now, _tuningString, _tuningAttempt,
                                   measured, target,
                                   _motors.getServoAngle(_tuningString));

            printf("  [TUNING] String %d attempt %d: %.1f Hz (target %.1f, err %.1f)\n",
                   _tuningString, _tuningAttempt, measured, target, error);

            if (std::abs(error) <= TUNING_PITCH_THRESHOLD_HZ) {
                printf("  [TUNING] String %d in tune.\n", _tuningString);
                _motors.commandDamperEngage(_tuningString);
                _tuningString++;
                _tuningSub = TuningSubState::SELECT_STRING;
            } else if (_tuningAttempt >= TUNING_MAX_ATTEMPTS) {
                printf("  [TUNING] String %d: attempt cap reached. Moving on.\n", _tuningString);
                _motors.commandDamperEngage(_tuningString);
                _tuningString++;
                _tuningSub = TuningSubState::SELECT_STRING;
            } else {
                float currentAngle = _motors.getServoAngle(_tuningString);
                float adjustment   = (error > 0.0f) ? -TUNING_ANGLE_STEP : TUNING_ANGLE_STEP;
                float newAngle     = currentAngle + adjustment;
                newAngle = (newAngle < 10.0f)  ? 10.0f  : newAngle;
                newAngle = (newAngle > 170.0f) ? 170.0f : newAngle;
                _motors.commandTuningServo(_tuningString, newAngle);
                _tuningAttempt++;
                _tuningPluckCount = 0;
                _tuningSub = TuningSubState::COMMANDING_SERVO;
            }
            break;
        }

        case TuningSubState::DONE:
            break;
    }
}

// -----------------------------------------------------------------------------
// _duringPerforming — plays composed sequence, one note per stage
// -----------------------------------------------------------------------------
void FSM::_duringPerforming() {
    uint32_t now    = hal_millis();
    int      seqLen = _composer.getLength();

    // Guard: already fired completion event, waiting for transition.
    if (_perfNoteIdx > seqLen) return;

    if (_perfNoteIdx == seqLen) {
        if (seqLen == 0) {
            printf("  [PERFORMING] No sequence loaded. Firing EVT_DONE.\n");
        } else {
            printf("  [PERFORMING] Sequence complete. Firing EVT_DONE.\n");
        }
        _perfNoteIdx = seqLen + 1;
        _detector.injectDone();
        return;
    }

    const Note& note = _composer.getSequence()[_perfNoteIdx];

    if (!_perfFretDone) {
        for (int s = 0; s < NUM_STRINGS; s++) _motors.commandDamperEngage(s);
        _motors.commandDamperLift(note.string_id);
        _motors.commandFret(note.string_id, note.note_index);
        if (_motors.isFretAtTarget(note.string_id) &&
            _motors.isDamperAtTarget(note.string_id)) {
            _perfFretDone = true;
        }
        return;
    }

    if (!_perfWaitingPluck) {
        _motors.commandPluck(note.string_id);
        _perfPluckStart   = now;
        _perfWaitingPluck = true;
        return;
    }

    if (_perfWaitingPluck && (now - _perfPluckStart) < PERFORM_PLUCK_HOLD_MS) {
        return;
    }

    if (!_perfWaitingGap) {
        _motors.commandFretLift(note.string_id);
        _motors.commandDamperEngage(note.string_id);
        _perfGapStart     = now;
        _perfWaitingGap   = true;
        _perfWaitingPluck = false;
        return;
    }

    if ((now - _perfGapStart) >= PERFORM_NOTE_GAP_MS) {
        printf("  [PERFORMING] Note %d done (string %d, note_idx %d).\n",
               _perfNoteIdx, note.string_id, note.note_index);
        hal_telemetryLogNote(now, note.note_index, note.string_id, "PERFORMING");
        _motors.commandDamperLift(note.string_id);
        _perfNoteIdx++;
        _perfFretDone     = false;
        _perfWaitingPluck = false;
        _perfWaitingGap   = false;
    }
}

// -----------------------------------------------------------------------------
// _duringPreComposing — interactive note audition
// -----------------------------------------------------------------------------
void FSM::_duringPreComposing() {
    uint32_t now = hal_millis();

    switch (_preCompSub) {

        case PreCompSubState::MENU_NAVIGATE:
            break;

        case PreCompSubState::NOTE_COMMANDING:
            for (int s = 0; s < NUM_STRINGS; s++) _motors.commandDamperEngage(s);
            _motors.commandDamperLift(_preCompStringIdx);
            _motors.commandFret(_preCompStringIdx, _preCompNoteIdx);
            if (_motors.isFretAtTarget(_preCompStringIdx) &&
                _motors.isDamperAtTarget(_preCompStringIdx)) {
                _preCompSub = PreCompSubState::NOTE_PLAYING;
            }
            break;

        case PreCompSubState::NOTE_PLAYING:
            if (_preCompPluckStart == 0) {
                _motors.commandPluck(_preCompStringIdx);
                _preCompPluckStart = now;
            } else if ((now - _preCompPluckStart) >= PRECOMP_PLUCK_HOLD_MS) {
                _motors.commandFretLift(_preCompStringIdx);
                _motors.commandDamperEngage(_preCompStringIdx);

                // Seed Markov: record the transition from the previous note.
                _composer.seed(_preCompLastNote, _preCompNoteIdx);
                _preCompLastNote = _preCompNoteIdx;

                _preCompSub = PreCompSubState::NOTE_IDLE;
            }
            break;

        case PreCompSubState::NOTE_IDLE:
            hal_telemetryLogNote(now, _preCompNoteIdx, _preCompStringIdx, "PRE_COMPOSING");
            printf("  [PRE_COMPOSING] Note %d on string %d played. Back to menu.\n",
                   _preCompNoteIdx, _preCompStringIdx);
            printf("  [PRE_COMPOSING] Note: %d  String: %d  |  t =scroll string  x=scroll note  n=play  C=compose\n",
                   _preCompNoteIdx, _preCompStringIdx);
            _preCompPluckStart = 0;
            _preCompSub = PreCompSubState::MENU_NAVIGATE;
            break;
    }
}

// -----------------------------------------------------------------------------
// onExit
// -----------------------------------------------------------------------------
void FSM::onExit(State s) {
    printf("[FSM] <-- Exiting: %s\n", stateName(s));
    if (s == State::SLEEP) {
        _motors.energizeAll();
        printf("  [SLEEP] Motors re-energized.\n");
    }
}

// -----------------------------------------------------------------------------
// computeNextState
// -----------------------------------------------------------------------------
State FSM::computeNextState(State s, Event evt) {

    if (evt == Event::EVT_FAULT) return State::ERROR_STATE;

    switch (s) {

        case State::HOMING:
            if (evt == Event::EVT_BTN_STOP) return State::IDLE;
            if (evt == Event::EVT_DONE)     return State::IDLE;
            break;

        case State::IDLE:
            if (evt == Event::EVT_BTN_COMPOSE)     return State::COMPOSING;
            if (evt == Event::EVT_IDLE_TIMEOUT)    return State::SLEEP;
            if (evt == Event::EVT_BTN_TUNE) {
                if (_homingComplete) return State::TUNING;
                printf("  [IDLE] Home first!\n");
            }
            if (evt == Event::EVT_BTN_PRE_COMPOSE) {
                if (_homingComplete) return State::PRE_COMPOSING;
                printf("  [IDLE] Home first!\n");
                return State::IDLE;
            }
            if (evt == Event::EVT_BTN_PERFORM) {
                if (_homingComplete) return State::PERFORMING;
                printf("  [IDLE] Home first!\n");
                return State::IDLE;
            }
            break;

        case State::SLEEP:
            if (evt == Event::EVT_BTN_WAKE) return State::IDLE;
            break;

        case State::TUNING:
            if (evt == Event::EVT_BTN_STOP) return State::IDLE;
            if (evt == Event::EVT_DONE)     return State::IDLE;
            break;

        case State::PRE_COMPOSING:
            if (evt == Event::EVT_BTN_STOP)     return State::IDLE;
            if (evt == Event::EVT_BTN_COMPOSE)  return State::COMPOSING;
            if (evt == Event::EVT_IDLE_TIMEOUT) return State::SLEEP;
            if (evt == Event::EVT_NOTE_PLAY) {
                _preCompSub        = PreCompSubState::NOTE_COMMANDING;
                _preCompPluckStart = 0;
                return State::PRE_COMPOSING;
            }
            if (evt == Event::EVT_BTN_PERFORM) {
                _preCompNoteIdx = (_preCompNoteIdx + 1) % 12;
                printf("  [PRE_COMPOSING] Note: %d  String: %d\n",
                       _preCompNoteIdx, _preCompStringIdx);
                return State::PRE_COMPOSING;
            }
            if (evt == Event::EVT_BTN_TUNE) {
                _preCompStringIdx = (_preCompStringIdx + 1) % NUM_STRINGS;
                printf("  [PRE_COMPOSING] Note: %d  String: %d\n",
                       _preCompNoteIdx, _preCompStringIdx);
                return State::PRE_COMPOSING;
            }
            break;

        case State::COMPOSING:
            if (evt == Event::EVT_BTN_STOP)     return State::IDLE;
            if (evt == Event::EVT_BTN_PERFORM)  return State::PERFORMING;
            if (evt == Event::EVT_IDLE_TIMEOUT) return State::SLEEP;
            if (evt == Event::EVT_DONE)         return State::PERFORMING;
            break;

        case State::PERFORMING:
            if (evt == Event::EVT_BTN_STOP) return State::IDLE;
            if (evt == Event::EVT_DONE)     return State::IDLE;
            break;

        case State::ERROR_STATE:
            if (evt == Event::EVT_BTN_STOP) return State::IDLE;
            break;
    }

    return s;
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
State FSM::getState() const { return _state; }

const char* FSM::stateName(State s) {
    switch (s) {
        case State::HOMING:        return "HOMING";
        case State::IDLE:          return "IDLE";
        case State::SLEEP:         return "SLEEP";
        case State::PRE_COMPOSING: return "PRE_COMPOSING";
        case State::COMPOSING:     return "COMPOSING";
        case State::TUNING:        return "TUNING";
        case State::PERFORMING:    return "PERFORMING";
        case State::ERROR_STATE:   return "ERROR";
        default:                   return "UNKNOWN";
    }
}

const char* FSM::eventName(Event evt) {
    switch (evt) {
        case Event::EVT_NONE:            return "NONE";
        case Event::EVT_FAULT:           return "FAULT";
        case Event::EVT_BTN_STOP:        return "BTN_STOP";
        case Event::EVT_BTN_WAKE:        return "BTN_WAKE";
        case Event::EVT_BTN_COMPOSE:     return "BTN_COMPOSE";
        case Event::EVT_BTN_PRE_COMPOSE: return "BTN_PRE_COMPOSE";
        case Event::EVT_BTN_TUNE:        return "BTN_TUNE";
        case Event::EVT_BTN_PERFORM:     return "BTN_PERFORM";
        case Event::EVT_NOTE_PLAY:       return "NOTE_PLAY";
        case Event::EVT_DONE:            return "DONE";
        case Event::EVT_IDLE_TIMEOUT:    return "IDLE_TIMEOUT";
        default:                         return "UNKNOWN";
    }
}