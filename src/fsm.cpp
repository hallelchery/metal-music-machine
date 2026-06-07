#include "fsm.h"
#include "../hal/hal.h"
#include <cstdio>

// -----------------------------------------------------------------------
// Constructor — set initial values
// -----------------------------------------------------------------------
FSM::FSM() : current_state(State::HOMING), homing_complete(false) {
    // Nothing else needed here
}

// -----------------------------------------------------------------------
// begin() — called once at startup
// -----------------------------------------------------------------------
void FSM::begin() {
    printf("[FSM] Starting up. Entering HOMING.\n");
    onEnter(current_state);
}

// -----------------------------------------------------------------------
// update() — the heart of the FSM, called every loop pass
// -----------------------------------------------------------------------
void FSM::update(Event evt) {
    // EVT_NONE means nothing happened. No work to do.
    if (evt == Event::EVT_NONE) return;

    State next = computeNextState(current_state, evt);

    // Only transition if the state is actually changing
    if (next != current_state) {
        onExit(current_state);
        current_state = next;
        onEnter(current_state);
    }

    // Log this event to telemetry regardless of whether a transition occurred
    hal_telemetryLog(hal_millis(), stateName(current_state), eventName(evt));
}

// -----------------------------------------------------------------------
// onEnter() — runs once when we ARRIVE in a state
// -----------------------------------------------------------------------
void FSM::onEnter(State s) {
    printf("[FSM] --> Entering: %s\n", stateName(s));
    hal_displayPrint(stateName(s));

    switch (s) {
        case State::HOMING:
            printf("  [HOMING] Driving motors to reference positions...\n");
            break;
        case State::IDLE:
            if (homing_complete) {
                printf("  [IDLE] Ready. Press C=Compose, P=Pre-compose, T=Tune, X=Perform.\n");
            } else {
                printf("  [IDLE] Homing not complete. TUNE and PERFORM are locked.\n");
            }
            break;
        case State::SLEEP:
            printf("  [SLEEP] Going to sleep. Motors de-energized.\n");
            break;
        case State::PRE_COMPOSING:
            printf("  [PRE_COMPOSING] Navigate notes. N=play note.\n");
            break;
        case State::COMPOSING:
            printf("  [COMPOSING] Running Markov chain composer...\n");
            break;
        case State::TUNING:
            printf("  [TUNING] Calibrating string tensions...\n");
            break;
        case State::PERFORMING:
            printf("  [PERFORMING] Playing note sequence.\n");
            break;
        case State::ERROR_STATE:
            printf("  [ERROR] FAULT DETECTED. Hold S for 2 seconds to reset.\n");
            break;
    }
}

// -----------------------------------------------------------------------
// onExit() — runs once when we LEAVE a state
// -----------------------------------------------------------------------
void FSM::onExit(State s) {
    printf("[FSM] <-- Exiting: %s\n", stateName(s));

    // Special logic: leaving HOMING successfully sets the guard flag
    if (s == State::HOMING) {
        // onExit is called just before we transition.
        // We'll set homing_complete in computeNextState for precision,
        // but we could also handle it here.
    }
}

// -----------------------------------------------------------------------
// computeNextState() — the full transition table from the spec
// -----------------------------------------------------------------------
State FSM::computeNextState(State s, Event evt) {

    // EVT_FAULT is highest priority — always wins, from any state
    if (evt == Event::EVT_FAULT) return State::ERROR_STATE;

    switch (s) {

        case State::HOMING:
            if (evt == Event::EVT_BTN_STOP) return State::IDLE;    // homing_complete stays false
            if (evt == Event::EVT_DONE) {
                homing_complete = true;  // Successful homing!
                return State::IDLE;
            }
            break;

        case State::IDLE:
            if (evt == Event::EVT_BTN_COMPOSE)      return State::COMPOSING;
            if (evt == Event::EVT_IDLE_TIMEOUT)     return State::SLEEP;
            if (evt == Event::EVT_BTN_TUNE) {
                if (homing_complete) return State::TUNING; // Guard necessary for fret calibration
                printf("  [IDLE] Home first! Homing not complete.\n");
            } 
            if (evt == Event::EVT_BTN_PRE_COMPOSE) {
                if (homing_complete) return State::PRE_COMPOSING;
                printf("  [IDLE] Home first! Homing not complete.\n");
                return State::IDLE;  // Stay put — guard blocked transition
            }
            if (evt == Event::EVT_BTN_PERFORM) {
                if (homing_complete) return State::PERFORMING;
                printf("  [IDLE] Home first! Homing not complete.\n");
                return State::IDLE;
            }
            break;

        case State::SLEEP:
            if (evt == Event::EVT_BTN_WAKE) return State::IDLE;
            break;

        case State::TUNING:
            if (evt == Event::EVT_BTN_STOP)         return State::IDLE;
            if (evt == Event::EVT_DONE)             return State::IDLE;
            break;

        case State::PRE_COMPOSING:
            if (evt == Event::EVT_BTN_STOP)         return State::IDLE;
            if (evt == Event::EVT_BTN_COMPOSE)      return State::COMPOSING;
            if (evt == Event::EVT_IDLE_TIMEOUT)     return State::SLEEP;
            if (evt == Event::EVT_NOTE_PLAY) {
                printf("  [PRE_COMPOSING] Note played (stub).\n");
                return State::PRE_COMPOSING;  // Stay; real motor work added later
            }
            break;

        case State::COMPOSING:
            if (evt == Event::EVT_BTN_STOP)         return State::IDLE;
            if (evt == Event::EVT_BTN_PERFORM)      return State::PERFORMING;
            if (evt == Event::EVT_IDLE_TIMEOUT)     return State::SLEEP;
            if (evt == Event::EVT_DONE)             return State::PERFORMING;
            break;

        case State::PERFORMING:
            if (evt == Event::EVT_BTN_STOP)         return State::IDLE;
            if (evt == Event::EVT_DONE)             return State::IDLE;
            break;

        case State::ERROR_STATE:
            // Hold-to-reset: for now, a single STOP press resets (hold logic comes in Iter 1)
            if (evt == Event::EVT_BTN_STOP)         return State::IDLE;
            break;
    }

    return s; // Default: stay in current state
}

// -----------------------------------------------------------------------
// Helpers — human-readable names for logging
// -----------------------------------------------------------------------
State FSM::getState() const { return current_state; }

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