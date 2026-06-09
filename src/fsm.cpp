#include "fsm.h"
#include "event_detector.h"
#include "../hal/hal.h"
#include <cstdio>

FSM::FSM(MotorController& controller, EventDetector& detector)
    : _state(State::HOMING)
    , _homingComplete(false)
    , _motors(controller)
    , _detector(detector)
{}

void FSM::begin() {
    printf("[FSM] Starting up. Entering HOMING.\n");
    onEnter(_state);
}

void FSM::update(Event evt) {
    // Run per-state continuous logic before processing any event.
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
// onEnter — runs once when a state is entered
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
                printf("  [IDLE] Ready. Press C=Compose, P=Pre-compose, T=Tune, X=Perform.\n");
            } else {
                printf("  [IDLE] Homing not complete. TUNE and PERFORM are locked.\n");
            }
            break;

        case State::SLEEP:
            printf("  [SLEEP] Entering low-power standby. Motors de-energized.\n");
            _motors.deenergizeAll();
            break;

        case State::PRE_COMPOSING:
            printf("  [PRE_COMPOSING] Navigate notes. N=play note.\n");
            break;

        case State::COMPOSING:
            printf("  [COMPOSING] Running Markov chain composer...\n");
            break;

        case State::TUNING:
            printf("  [TUNING] Calibrating string tensions...\n");
            _motors.startTuning();
            break;

        case State::PERFORMING:
            printf("  [PERFORMING] Playing note sequence.\n");
            break;

        case State::ERROR_STATE:
            printf("  [ERROR] FAULT DETECTED. Hold S for 2 seconds to reset.\n");
            break;
    }
}

// -----------------------------------------------------------------------------
// during — runs every loop pass while inside a state
// -----------------------------------------------------------------------------
void FSM::during(State s) {
    switch (s) {
        case State::HOMING:
            _motors.tick(hal_millis());
            if (_motors.isFaulted()) {
                _detector.injectFault();
            } else if (_motors.isHomingComplete()) {
                printf("  [HOMING] All steppers at reference. Homing complete.\n");
                _homingComplete = true;
                _detector.injectDone();
            }
            break;

        case State::TUNING:
            _motors.tick(hal_millis());
            if (_motors.isFaulted()) {
                _detector.injectFault();
            } else if (_motors.isTuningComplete()) {
                printf("  [TUNING] All steppers at calibration positions. Tuning complete.\n");
                _detector.injectDone();
            }
            break;

        case State::SLEEP:
            // Motors remain de-energized; nothing to tick.
            break;

        default:
            break;
    }
}

// -----------------------------------------------------------------------------
// onExit — runs once when a state is left
// -----------------------------------------------------------------------------
void FSM::onExit(State s) {
    printf("[FSM] <-- Exiting: %s\n", stateName(s));

    if (s == State::SLEEP) {
        _motors.energizeAll();
        printf("  [SLEEP] Motors re-energized.\n");
    }
}

// -----------------------------------------------------------------------------
// computeNextState — full transition table
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
            if (evt == Event::EVT_BTN_STOP)    return State::IDLE;
            if (evt == Event::EVT_BTN_COMPOSE) return State::COMPOSING;
            if (evt == Event::EVT_IDLE_TIMEOUT) return State::SLEEP;
            if (evt == Event::EVT_NOTE_PLAY) {
                printf("  [PRE_COMPOSING] Note played (stub).\n");
                return State::PRE_COMPOSING;
            }
            break;

        case State::COMPOSING:
            if (evt == Event::EVT_BTN_STOP)    return State::IDLE;
            if (evt == Event::EVT_BTN_PERFORM) return State::PERFORMING;
            if (evt == Event::EVT_IDLE_TIMEOUT) return State::SLEEP;
            if (evt == Event::EVT_DONE)        return State::PERFORMING;
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