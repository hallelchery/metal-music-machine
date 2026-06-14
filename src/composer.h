#pragma once
#include <cstdint>
#include <cstring>

static constexpr int NOTE_COUNT    = 12;
static constexpr int MAX_SEQ_LEN   = 64;

// A note in the composed sequence.
struct Note {
    int string_id;
    int note_index;
};

// Markov chain composer.
//
// Holds a 12x12 transition probability table. PRE_COMPOSING seeds weights
// by incrementing table entries for observed transitions. COMPOSING calls
// step() once per loop pass until the NoteSequence buffer is full.
//
// Memory footprint: 144 floats = 576 bytes. Well within Teensy 4.0's 1 MB RAM.
class Composer {
public:
    Composer();

    // Seed a transition: increments the weight from 'from' to 'to'.
    // Called by PRE_COMPOSING each time a note is auditioned.
    void seed(int from_note, int to_note);

    // Reset the sequence buffer and internal generation state.
    // Called at the start of each COMPOSING run.
    void beginComposing(int string_for_new_notes);

    // Generate one note and append it to the sequence.
    // Returns true when the sequence buffer is full (EVT_DONE should fire).
    // Non-blocking: returns false if not yet done.
    bool step();

    // Access the completed note sequence.
    const Note* getSequence()   const { return _sequence; }
    int         getLength()     const { return _seqLen; }

    // Log the note distribution over the sequence to stdout.
    void logDistribution() const;

private:
    float _table[NOTE_COUNT][NOTE_COUNT];   // transition weights (not normalized)
    Note  _sequence[MAX_SEQ_LEN];
    int   _seqLen;
    int   _currentNote;
    int   _composingString;   // which string to assign generated notes to (rotates)

    // Pick the next note using weighted random selection from _table[current].
    int _pickNext(int from_note) const;

    // Normalize a row so weights sum to 1.0 (used internally by _pickNext).
    // Falls back to uniform distribution if the row sums to zero.
    float _rowSum(int row) const;
};