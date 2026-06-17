#include "event_detector.h"

EventDetector::EventDetector(uint32_t idleTimeoutMs)
    : _faultPending(false)
    , _donePending(false)
    , _notePending(false)
    , _composePending(false)
    , _preComposePending(false)
    , _tunePending(false)
    , _performPending(false)
    , _stopPending(false)
    , _wakePending(false)
    , _idleTimeoutPending(false)
    , _idleTimeoutMs(idleTimeoutMs)
    , _lastActivityMs(0)
{
    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        _rawButtons[i] = false;
    }
}

void EventDetector::setButtonRaw(ButtonId btn, bool pressed) {
    _rawButtons[btn] = pressed;
}

void EventDetector::injectFault()       { _faultPending       = true; }
void EventDetector::injectDone()        { _donePending        = true; }
void EventDetector::injectNotePending() { _notePending        = true; }
void EventDetector::injectStop()        { _stopPending        = true; }
void EventDetector::injectCompose()     { _composePending     = true; }
void EventDetector::injectPreCompose()  { _preComposePending  = true; }
void EventDetector::injectTune()        { _tunePending        = true; }
void EventDetector::injectPerform()     { _performPending     = true; }
void EventDetector::injectWake()        { _wakePending        = true; }
void EventDetector::injectIdleTimeout() { _idleTimeoutPending = true; }

void EventDetector::resetIdleTimer(uint32_t nowMs) {
    _lastActivityMs = nowMs;
}

void EventDetector::update(State currentState, uint32_t nowMs, EventQueue& queue) {

    for (uint8_t i = 0; i < BTN_COUNT; i++) {
        _debouncers[i].update(_rawButtons[i], nowMs);
        _rawButtons[i] = false;
    }

    // PRIORITY 1: Hardware fault
    if (_faultPending) {
        _faultPending = false;
        queue.push(Event::EVT_FAULT);
        resetIdleTimer(nowMs);
        return;
    }

    // PRIORITY 2: STOP
    if (_stopPending) {
        _stopPending = false;
        queue.push(Event::EVT_BTN_STOP);
        resetIdleTimer(nowMs);
        return;
    }

    // PRIORITY 3: WAKE — only in SLEEP
    if (currentState == State::SLEEP && _wakePending) {
        _wakePending = false;
        queue.push(Event::EVT_BTN_WAKE);
        resetIdleTimer(nowMs);
        return;
    }

    // PRIORITY 4: UI buttons
    if (_composePending) {
        _composePending = false;
        queue.push(Event::EVT_BTN_COMPOSE);
        resetIdleTimer(nowMs);
        return;
    }
    if (_preComposePending) {
        _preComposePending = false;
        queue.push(Event::EVT_BTN_PRE_COMPOSE);
        resetIdleTimer(nowMs);
        return;
    }
    if (_tunePending) {
        _tunePending = false;
        queue.push(Event::EVT_BTN_TUNE);
        resetIdleTimer(nowMs);
        return;
    }
    if (_performPending) {
        _performPending = false;
        queue.push(Event::EVT_BTN_PERFORM);
        resetIdleTimer(nowMs);
        return;
    }

    // PRIORITY 5: Note play
    if (_notePending) {
        _notePending = false;
        queue.push(Event::EVT_NOTE_PLAY);
        resetIdleTimer(nowMs);
        return;
    }

    // PRIORITY 6: Internal task completion
    if (_donePending) {
        _donePending = false;
        queue.push(Event::EVT_DONE);
        resetIdleTimer(nowMs);
        return;
    }

    // PRIORITY 7: Injected idle timeout (simulator 'i' key) - state-gated same as natural
    if (_idleTimeoutPending) {
        if (currentState == State::IDLE         ||
            currentState == State::PRE_COMPOSING ||
            currentState == State::COMPOSING)
        {
            _idleTimeoutPending = false;
            queue.push(Event::EVT_IDLE_TIMEOUT);
            resetIdleTimer(nowMs);
        } else {
            _idleTimeoutPending = false;  // discard if in non-idle-eligible state
        }
        return;
    }
    
    // PRIORITY 8: Natural idle timeout
    if (currentState == State::IDLE         ||
        currentState == State::PRE_COMPOSING ||
        currentState == State::COMPOSING)
    {
        if ((nowMs - _lastActivityMs) >= _idleTimeoutMs) {
            queue.push(Event::EVT_IDLE_TIMEOUT);
            resetIdleTimer(nowMs);
        }
    }
}