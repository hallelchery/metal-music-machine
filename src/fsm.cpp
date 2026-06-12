#include "fsm.h"
#include "event_detector.h"
#include "../hal/hal.h"
#include <cstdio>
#include <cmath>

// Target pitch per string (Hz). Matched to the sim physics model base frequencies.
static constexpr float TARGET_HZ[3] = { 196.0f, 246.9f, 293.7f };

FSM::FSM(MotorController& controller, EventDetector& detector)
    : _state(State::HOMING)
    , _homingComplete(false)
    , _motors(controller)
    , _detector(detector)
    // TUNING
    , _tuningSub(TuningSubState::SELECT_STRING)
    , _tuningString(0)
    , _tuningAttempt(0)
    , _tuningPluckCount(0)
    , _tuningSettleStart(0)
    , _tuningPitchSum(0.0f)
    , _tuningReadCount(0)
    // PERFORMING
    , _perfNoteIdx(0)
    , _perfPluckStart(0)
    , _perfGapStart(0)
    , _perfWaitingPluck(false)
    , _perfWaitingGap(false)
    , _perfFretDone(false)
    // PRE_COMPOSING
    , _preCompSub(PreCompSubState::MENU_NAVIGATE)
    , _preCompNoteIdx(0)
    , _preCompStringIdx(0)
    , _preCompPluckStart(0)
{
    for (int i = 0; i < 3; i++) _tuningTargetHz[i] = TARGET_HZ[i];
}

void FSM::begin() {
    printf("[FSM] Starting up. Entering HOMING.\n");
    onEnter(_state);
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
            printf("  [SLEEP] Entering low-power standby. Motors de-energized.\n");
            _motors.deenergizeAll();
            break;

        case State::PRE_COMPOSING:
            _preCompSub      = PreCompSubState::MENU_NAVIGATE;
            _preCompNoteIdx  = 0;
            _preCompStringIdx= 0;
            printf("  [PRE_COMPOSING] Note: %d  String: %d  |  UP/DOWN=scroll  N=play  C=compose\n",
                   _preCompNoteIdx, _preCompStringIdx);
            break;

        case State::COMPOSING:
            printf("  [COMPOSING] Running Markov chain composer (stub)...\n");
            break;

        case State::TUNING:
            printf("  [TUNING] Starting closed-loop pitch calibration...\n");
            _tuningSub    = TuningSubState::SELECT_STRING;
            _tuningString = 0;
            _tuningAttempt= 0;
            break;

        case State::PERFORMING:
            printf("  [PERFORMING] Playing hardcoded note sequence (%d notes).\n", NOTE_SEQ_LEN);
            _perfNoteIdx      = 0;
            _perfWaitingPluck = false;
            _perfWaitingGap   = false;
            _perfFretDone     = false;
            break;

        case State::ERROR_STATE:
            printf("  [ERROR] FAULT DETECTED. Hold S for 2 seconds to reset.\n");
            break;
    }
}

// -----------------------------------------------------------------------------
// during — per-state continuous logic, runs every loop pass
// -----------------------------------------------------------------------------
void FSM::during(State s) {
    _motors.tick(hal_millis());

    if (_motors.isFaulted() && s != State::ERROR_STATE) {
        _detector.injectFault();
        return;
    }

    switch (s) {
        case State::HOMING:       _duringHoming();       break;
        case State::TUNING:       _duringTuning();       break;
        case State::PERFORMING:   _duringPerforming();   break;
        case State::PRE_COMPOSING:_duringPreComposing(); break;
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
// _duringTuning — closed-loop pitch correction sub-state machine
// -----------------------------------------------------------------------------
void FSM::_duringTuning() {
    uint32_t now = hal_millis();

    switch (_tuningSub) {

        case TuningSubState::SELECT_STRING:
            if (_tuningString >= NUM_STRINGS) {
                // All strings processed.
                printf("  [TUNING] All strings tuned. Firing EVT_DONE.\n");
                _tuningSub = TuningSubState::DONE;
                _detector.injectDone();
                return;
            }
            hal_selectTuningString(_tuningString);
            _tuningAttempt  = 0;
            _tuningPitchSum = 0.0f;
            _tuningReadCount= 0;
            _tuningPluckCount = 0;
            printf("  [TUNING] String %d — target %.1f Hz\n",
                   _tuningString, _tuningTargetHz[_tuningString]);
            _tuningSub = TuningSubState::COMMANDING_SERVO;
            break;

        case TuningSubState::COMMANDING_SERVO:
            // Wait for the tuning servo to reach its current target.
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
                // All plucks fired; start the mic settle timer.
                _tuningSettleStart = now;
                _tuningPitchSum    = 0.0f;
                _tuningReadCount   = 0;
                _tuningSub = TuningSubState::LISTENING;
            }
            break;

        case TuningSubState::LISTENING:
            if ((now - _tuningSettleStart) >= TUNING_MIC_SETTLE_MS) {
                // Settle time elapsed; take a pitch reading.
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
                _tuningString++;
                _tuningSub = TuningSubState::SELECT_STRING;
            } else if (_tuningAttempt >= TUNING_MAX_ATTEMPTS) {
                printf("  [TUNING] String %d: attempt cap reached. Moving on.\n", _tuningString);
                _tuningString++;
                _tuningSub = TuningSubState::SELECT_STRING;
            } else {
                // Adjust servo angle proportional to error direction.
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
            // Waiting for the FSM transition triggered by injectDone().
            break;
    }
}

// -----------------------------------------------------------------------------
// _duringPerforming — plays one note per stage, advances through NOTE_SEQUENCE
// -----------------------------------------------------------------------------
void FSM::_duringPerforming() {
    uint32_t now = hal_millis();

    if (_perfNoteIdx >= NOTE_SEQ_LEN) {
        printf("  [PERFORMING] Sequence complete. Firing EVT_DONE.\n");
        _detector.injectDone();
        _perfNoteIdx = NOTE_SEQ_LEN + 1; // prevent re-firing
        return;
    }

    const Note& note = NOTE_SEQUENCE[_perfNoteIdx];

    // Stage 1: command fret (stepper + servo) and wait for arrival.
    if (!_perfFretDone) {
        _motors.commandFret(note.string_id, note.note_index);
        if (_motors.isFretAtTarget(note.string_id)) {
            _perfFretDone = true;
        }
        return;
    }

    // Stage 2: fire pluck and start hold timer.
    if (!_perfWaitingPluck) {
        _motors.commandPluck(note.string_id);
        _perfPluckStart   = now;
        _perfWaitingPluck = true;
        return;
    }

    // Stage 3: wait for pluck hold timer.
    if (_perfWaitingPluck && (now - _perfPluckStart) < PERFORM_PLUCK_HOLD_MS) {
        return;
    }

    // Stage 4: lift fret, start inter-note gap timer.
    if (!_perfWaitingGap) {
        _motors.commandFretLift(note.string_id);
        _perfGapStart    = now;
        _perfWaitingGap  = true;
        _perfWaitingPluck= false;
        return;
    }

    // Stage 5: wait for gap timer, then advance to next note.
    if ((now - _perfGapStart) >= PERFORM_NOTE_GAP_MS) {
        printf("  [PERFORMING] Note %d done (string %d, note_idx %d).\n",
               _perfNoteIdx, note.string_id, note.note_index);
        _perfNoteIdx++;
        _perfFretDone     = false;
        _perfWaitingPluck = false;
        _perfWaitingGap   = false;
    }
}

// -----------------------------------------------------------------------------
// _duringPreComposing — interactive note audition sub-state machine
// -----------------------------------------------------------------------------
void FSM::_duringPreComposing() {
    uint32_t now = hal_millis();

    switch (_preCompSub) {

        case PreCompSubState::MENU_NAVIGATE:
            // User scrolls note/string selection; input handled via EVT_NOTE_PLAY
            // in computeNextState. Nothing to tick here.
            break;

        case PreCompSubState::NOTE_COMMANDING:
            _motors.commandFret(_preCompStringIdx, _preCompNoteIdx);
            if (_motors.isFretAtTarget(_preCompStringIdx)) {
                _tuningSub = TuningSubState::DONE; // harmless reuse guard
                _preCompSub = PreCompSubState::NOTE_PLAYING;
            }
            break;

        case PreCompSubState::NOTE_PLAYING:
            if (_preCompPluckStart == 0) {
                _motors.commandPluck(_preCompStringIdx);
                _preCompPluckStart = now;
            } else if ((now - _preCompPluckStart) >= PRECOMP_PLUCK_HOLD_MS) {
                _motors.commandFretLift(_preCompStringIdx);
                _preCompSub = PreCompSubState::NOTE_IDLE;
            }
            break;

        case PreCompSubState::NOTE_IDLE:
            // Log the note selection and return to menu.
            hal_telemetryLogNote(now, _preCompNoteIdx, _preCompStringIdx);
            printf("  [PRE_COMPOSING] Note %d on string %d played. Back to menu.\n",
                   _preCompNoteIdx, _preCompStringIdx);
            printf("  [PRE_COMPOSING] Note: %d  String: %d  |  UP/DOWN=scroll  N=play  C=compose\n",
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
                // Transition to NOTE_COMMANDING sub-state within PRE_COMPOSING.
                _preCompSub        = PreCompSubState::NOTE_COMMANDING;
                _preCompPluckStart = 0;
                return State::PRE_COMPOSING; // stay in same top-level state
            }
            // Scroll note selection.
            if (evt == Event::EVT_BTN_PERFORM) { // reuse PERFORM key as UP scroll
                _preCompNoteIdx = (_preCompNoteIdx + 1) % 12;
                printf("  [PRE_COMPOSING] Note: %d  String: %d\n",
                       _preCompNoteIdx, _preCompStringIdx);
                return State::PRE_COMPOSING;
            }
            if (evt == Event::EVT_BTN_TUNE) { // reuse TUNE key as string select
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