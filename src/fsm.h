#pragma once
#include "events.h"
#include "motor_controller.h"

// Forward declaration avoids a circular include between fsm.h and event_detector.h.
class EventDetector;

enum class State {
    HOMING,
    IDLE,
    SLEEP,
    PRE_COMPOSING,
    COMPOSING,
    TUNING,
    PERFORMING,
    ERROR_STATE  // "ERROR" is a reserved macro on some compilers
};

class FSM {
public:
    // Dependencies are injected at construction so the FSM can fire events
    // (via detector) and drive motors (via controller) from within during().
    FSM(MotorController& controller, EventDetector& detector);

    // Called once at startup to enter the initial state.
    void begin();

    // Called every loop pass. Runs during() for the current state, then
    // processes one event and potentially transitions.
    void update(Event evt);

    State       getState()          const;
    const char* stateName(State s);
    const char* eventName(Event evt);

private:
    State            _state;
    bool             _homingComplete;
    MotorController& _motors;
    EventDetector&   _detector;

    void onEnter(State s);
    void during(State s);
    void onExit(State s);
    State computeNextState(State s, Event evt);
};