#include "debouncer.h"

// Member initializer list: sets all private fields before the
// constructor body runs. This is the correct C++ way to initialize
// members — cleaner and sometimes faster than assigning in the body.
Debouncer::Debouncer()
    : _lastRaw(false)
    , _stableState(false)
    , _roseFlag(false)
    , _lastChangeMs(0)
{}

void Debouncer::update(bool rawSignal, uint32_t nowMs) {
    // If the raw signal changed from last pass, record when and wait.
    // We don't trust it yet — it might be bounce.
    if (rawSignal != _lastRaw) {
        _lastChangeMs = nowMs;
        _lastRaw = rawSignal;
    }

    // Only act once the signal has been stable for DEBOUNCE_MS.
    // (nowMs - _lastChangeMs) is how long since the last change.
    if ((nowMs - _lastChangeMs) >= DEBOUNCE_MS) {
        // Signal is stable. Did it arrive at a new confirmed value?
        if (rawSignal != _stableState) {
            _stableState = rawSignal;

            // We only care about the press (false→true), not the release.
            if (_stableState == true) {
                _roseFlag = true;
            }
        }
    }
}

bool Debouncer::rose() {
    // Read-and-clear: return the flag value, then immediately disarm it.
    // This guarantees the caller sees each press exactly once,
    // no matter how many loop passes happen before they call rose().
    if (_roseFlag) {
        _roseFlag = false;
        return true;
    }
    return false;
}