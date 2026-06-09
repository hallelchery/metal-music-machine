#pragma once
#include "events.h"

// The eight states of the Metal Music Machine.
enum class State {
    HOMING,
    IDLE,
    SLEEP,
    PRE_COMPOSING,
    COMPOSING,
    TUNING,
    PERFORMING,
    ERROR_STATE  // "ERROR" alone is a reserved macro in some compilers; renamed to be safe
};

class FSM {
public:
    FSM();

    // Called once at startup to enter the initial state.
    void begin();

    // Called every loop pass. Processes one event and potentially transitions.
    void update(Event evt);

    // Returns the current state (useful for testing and telemetry).
    State getState() const;

    // Helper: get a human-readable name for logging
    const char* stateName(State s);
    const char* eventName(Event evt);

private:
    State current_state;
    bool homing_complete; // Guards TUNE and PERFORM until homing has run successfully

    // Lifecycle methods — called automatically on transition
    void onEnter(State s);
    void onExit(State s);

    // The transition logic: given current state + event, what's the next state?
    State computeNextState(State s, Event evt);
};