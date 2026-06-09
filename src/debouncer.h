#pragma once
#include <cstdint>

// A Debouncer watches one boolean signal (one button).
// It ignores rapid on/off noise ("bounce") and only reports
// a clean LOW→HIGH transition — the moment a press is confirmed stable.

class Debouncer {
public:
    // DEBOUNCE_MS: how long the signal must be stable before we trust it.
    // 50ms is a classic embedded systems default — fast enough to feel
    // instant to a human, slow enough to outlast mechanical bounce.
    static constexpr uint32_t DEBOUNCE_MS = 50;

    Debouncer();

    // Call every loop pass with the raw signal and current time.
    void update(bool rawSignal, uint32_t nowMs);

    // Returns true exactly once after a stable rising edge.
    // Clears itself automatically — "read-and-clear" pattern.
    bool rose();

private:
    bool     _lastRaw;      // what we saw last loop pass
    bool     _stableState;  // the last confirmed stable value
    bool     _roseFlag;     // set when a clean press is detected
    uint32_t _lastChangeMs; // timestamp of the last raw signal change
};