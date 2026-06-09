#pragma once
#include <cstdint>
#include "events.h"
#include "debouncer.h"
#include "event_queue.h"
#include "fsm.h"   // for State enum

// EventDetector: reads raw inputs every loop pass, debounces them,
// and pushes the highest-priority pending event onto the queue.
//
// TERMS:
//   "State-gated"      — some events only make sense in certain states
//                        (e.g. WAKE only matters when we're in SLEEP)
//   "Priority-ordered" — when multiple inputs are ready simultaneously,
//                        more critical ones are enqueued first

class EventDetector {
public:
    // idleTimeoutMs: milliseconds of inactivity before EVT_IDLE_TIMEOUT fires.
    // Default is 10 seconds — easy to override in tests.
    explicit EventDetector(uint32_t idleTimeoutMs = 10000);

    // Call once per loop pass.
    // Reads raw button states, updates debouncers, detects the
    // highest-priority pending event, and pushes it to the queue.
    void update(State currentState, uint32_t nowMs, EventQueue& queue);

    // Simulation input: mark a button as pressed this pass.
    void setButtonRaw(ButtonId btn, bool pressed);

    // Simulation input: inject a hardware fault.
    void injectFault();

    // Simulation input: signal that an internal task completed (homing, etc.)
    void injectDone();

    // Simulation input: signal a note play request.
    void injectNotePending();

    // Simulation input: signal a stop request.
    void injectStop();

    // Simulation input: signal a compose request.
    void injectCompose();
    
    // Simulation input: signal a pre-compose request.
    void injectPreCompose();
    
    // Simulation input: signal a tune request.
    void injectTune();
    
    // Simulation input: signal a perform request.
    void injectPerform();
    
    // Simulation input: signal a wake request.
    void injectWake();

    // Resets the idle timer — called whenever any activity occurs.
    void resetIdleTimer(uint32_t nowMs);

private:
    Debouncer _debouncers[BTN_COUNT];  // one per button
    bool      _rawButtons[BTN_COUNT];  // current raw state per button

    bool _faultPending;
    bool _donePending;
    bool _notePending;
    bool _composePending;
    bool _preComposePending;
    bool _tunePending;
    bool _performPending;
    bool _stopPending;
    bool _wakePending;

    uint32_t _idleTimeoutMs;
    uint32_t _lastActivityMs;
};