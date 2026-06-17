#pragma once
#include <cstdint>
#include "events.h"
#include "debouncer.h"
#include "event_queue.h"
#include "fsm.h"

class EventDetector {
public:
    explicit EventDetector(uint32_t idleTimeoutMs = 30000);

    void update(State currentState, uint32_t nowMs, EventQueue& queue);

    void setButtonRaw(ButtonId btn, bool pressed);

    void injectFault();
    void injectDone();
    void injectNotePending();
    void injectStop();
    void injectCompose();
    void injectPreCompose();
    void injectTune();
    void injectPerform();
    void injectWake();
    void injectIdleTimeout();

    void resetIdleTimer(uint32_t nowMs);

private:
    Debouncer _debouncers[BTN_COUNT];
    bool      _rawButtons[BTN_COUNT];

    bool _faultPending;
    bool _donePending;
    bool _notePending;
    bool _composePending;
    bool _preComposePending;
    bool _tunePending;
    bool _performPending;
    bool _stopPending;
    bool _wakePending;
    bool _idleTimeoutPending;

    uint32_t _idleTimeoutMs;
    uint32_t _lastActivityMs;
};