#pragma once
#include "events.h"
#include "motor_controller.h"

class EventDetector;

enum class State {
    HOMING,
    IDLE,
    SLEEP,
    PRE_COMPOSING,
    COMPOSING,
    TUNING,
    PERFORMING,
    ERROR_STATE
};

// Sub-states for TUNING — run entirely within TUNING::during().
enum class TuningSubState {
    SELECT_STRING,
    COMMANDING_SERVO,
    PLUCKING,
    LISTENING,
    EVALUATING,
    DONE
};

// Sub-states for PRE_COMPOSING — run entirely within PRE_COMPOSING::during().
enum class PreCompSubState {
    MENU_NAVIGATE,
    NOTE_COMMANDING,
    NOTE_PLAYING,
    NOTE_IDLE
};

// Hardcoded note sequence played by PERFORMING (replaced by Markov output in Iter 4).
// Each entry is a {string_id, note_index} pair.
struct Note {
    int string_id;
    int note_index;
};

static constexpr int NOTE_SEQ_LEN = 8;
static constexpr Note NOTE_SEQUENCE[NOTE_SEQ_LEN] = {
    {0, 0}, {0, 2}, {1, 4}, {1, 5},
    {2, 7}, {2, 9}, {0, 11}, {1, 3}
};

// Tuning parameters.
static constexpr int   TUNING_MAX_ATTEMPTS       = 8;
static constexpr float TUNING_PITCH_THRESHOLD_HZ = 3.0f;
static constexpr int   TUNING_N_PLUCKS           = 2;
static constexpr uint32_t TUNING_MIC_SETTLE_MS   = 200;
static constexpr float TUNING_ANGLE_STEP         = 5.0f; // degrees per correction

// Performing timing.
static constexpr uint32_t PERFORM_PLUCK_HOLD_MS  = 150;  // pluck servo hold time
static constexpr uint32_t PERFORM_NOTE_GAP_MS    = 300;  // gap between notes

// PRE_COMPOSING timing.
static constexpr uint32_t PRECOMP_PLUCK_HOLD_MS  = 200;

class FSM {
public:
    FSM(MotorController& controller, EventDetector& detector);

    void begin();
    void update(Event evt);

    State       getState()          const;
    const char* stateName(State s);
    const char* eventName(Event evt);

private:
    State            _state;
    bool             _homingComplete;
    MotorController& _motors;
    EventDetector&   _detector;

    // --- TUNING sub-state ---
    TuningSubState _tuningSub;
    int            _tuningString;      // current string being tuned (0–2)
    int            _tuningAttempt;     // correction cycles for current string
    int            _tuningPluckCount;  // plucks fired this measurement
    uint32_t       _tuningSettleStart; // timestamp when settle timer started
    float          _tuningPitchSum;    // accumulated pitch readings for averaging
    int            _tuningReadCount;   // number of readings taken
    float          _tuningTargetHz[3]; // target pitch per string

    // --- PERFORMING sub-state ---
    int      _perfNoteIdx;       // current note in NOTE_SEQUENCE
    uint32_t _perfPluckStart;    // timestamp of last pluck command
    uint32_t _perfGapStart;      // timestamp when inter-note gap started
    bool     _perfWaitingPluck;  // true while waiting for pluck hold timer
    bool     _perfWaitingGap;    // true while waiting for inter-note gap
    bool     _perfFretDone;      // true once fret is at target for current note

    // --- PRE_COMPOSING sub-state ---
    PreCompSubState _preCompSub;
    int             _preCompNoteIdx;    // currently selected note index (0–11)
    int             _preCompStringIdx;  // currently selected string (0–2)
    uint32_t        _preCompPluckStart; // timestamp of last pluck command

    void onEnter(State s);
    void during(State s);
    void onExit(State s);
    State computeNextState(State s, Event evt);

    void _duringHoming();
    void _duringTuning();
    void _duringPerforming();
    void _duringPreComposing();
};