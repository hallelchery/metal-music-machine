#include "composer.h"
#include "motor_controller.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>

Composer::Composer()
    : _seqLen(0)
    , _currentNote(0)
    , _composingString(0)
{
    // Initialize table with a small uniform prior so unvisited transitions
    // are still reachable (avoids dead ends where a row sums to zero).
    for (int i = 0; i < NOTE_COUNT; i++)
        for (int j = 0; j < NOTE_COUNT; j++)
            _table[i][j] = 0.1f;

    memset(_sequence, 0, sizeof(_sequence));
}

void Composer::seed(int from_note, int to_note) {
    if (from_note < 0 || from_note >= NOTE_COUNT) return;
    if (to_note   < 0 || to_note   >= NOTE_COUNT) return;
    _table[from_note][to_note] += 1.0f;
}

void Composer::beginComposing(int string_for_new_notes) {
    _seqLen           = 0;
    _composingString  = string_for_new_notes % NUM_STRINGS;
    // Start from a random note each time so sequences don't all begin the same.
    _currentNote      = rand() % NOTE_COUNT;
    memset(_sequence, 0, sizeof(_sequence));
}

bool Composer::step() {
    if (_seqLen >= MAX_SEQ_LEN) return true;

    int next = _pickNext(_currentNote);
    _sequence[_seqLen].note_index = next;
    _sequence[_seqLen].string_id  = _composingString;
    _seqLen++;

    _currentNote       = next;
    _composingString   = (_composingString + 1) % NUM_STRINGS;

    return (_seqLen >= MAX_SEQ_LEN);
}

float Composer::_rowSum(int row) const {
    float sum = 0.0f;
    for (int j = 0; j < NOTE_COUNT; j++) sum += _table[row][j];
    return sum;
}

int Composer::_pickNext(int from_note) const {
    float sum = _rowSum(from_note);
    if (sum <= 0.0f) return rand() % NOTE_COUNT;

    // Draw a uniform random value in [0, sum) and walk the row until we
    // accumulate enough weight — this is the standard weighted sampling trick.
    float r = (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) * sum;
    float acc = 0.0f;
    for (int j = 0; j < NOTE_COUNT; j++) {
        acc += _table[from_note][j];
        if (r <= acc) return j;
    }
    return NOTE_COUNT - 1;  // fallback (floating-point rounding)
}

void Composer::logDistribution() const {
    int counts[NOTE_COUNT] = {};
    for (int i = 0; i < _seqLen; i++) counts[_sequence[i].note_index]++;
    printf("  [COMPOSER] Note distribution (%d notes):\n", _seqLen);
    for (int i = 0; i < NOTE_COUNT; i++) {
        printf("    note %2d: %d (%.1f%%)\n",
               i, counts[i],
               _seqLen > 0 ? 100.0f * counts[i] / _seqLen : 0.0f);
    }
}