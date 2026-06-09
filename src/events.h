#pragma once

// Every possible event the FSM can receive.
// An "event" is a signal that something happened — a button was pressed,
// a fault was detected, a task completed, etc.
enum class Event {
    EVT_NONE,           // Nothing happened this loop pass
    EVT_FAULT,          // Hardware fault (highest priority)
    EVT_BTN_STOP,       // STOP button pressed
    EVT_BTN_WAKE,       // Any button while in SLEEP
    EVT_BTN_COMPOSE,    // COMPOSE button
    EVT_BTN_PRE_COMPOSE,// PRE_COMPOSE button
    EVT_BTN_TUNE,       // TUNE button
    EVT_BTN_PERFORM,    // PERFORM button
    EVT_NOTE_PLAY,      // User confirmed a note in PRE_COMPOSING
    EVT_DONE,           // A task finished (homing, tuning, song end)
    EVT_IDLE_TIMEOUT,   // No input for N seconds → go to SLEEP
};

// Named indices for the physical buttons.
// Used to index into the Debouncer array in EventDetector.
// BTN_COUNT is a sentinel — it gives us the array size automatically.
// If a new button is created, add it here and BTN_COUNT updates automatically.
enum ButtonId {
    BTN_STOP = 0,
    BTN_COMPOSE,
    BTN_PRE_COMPOSE,
    BTN_TUNE,
    BTN_PERFORM,
    BTN_COUNT   // not a real button — just "how many buttons are there"
};