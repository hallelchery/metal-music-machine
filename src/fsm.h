#pragma once
#include "events.h"
#include "motor_controller.h"
#include "composer.h"

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

enum class TuningSubState {
    SELECT_STRING,
    COMMANDING_SERVO,
    PLUCKING,
    LISTENING,
    EVALUATING,
    DONE
};

enum class PreCompSubState {
    MENU_NAVIGATE,
    NOTE_COMMANDING,
    NOTE_PLAYING,
    NOTE_IDLE
};

// Tuning parameters.
static constexpr int      TUNING_MAX_ATTEMPTS       = 8;
static constexpr float    TUNING_PITCH_THRESHOLD_HZ = 3.0f;
static constexpr int      TUNING_N_PLUCKS           = 2;
static constexpr uint32_t TUNING_MIC_SETTLE_MS      = 200;
static constexpr float    TUNING_ANGLE_STEP         = 5.0f;

// Performing timing.
static constexpr uint32_t PERFORM_PLUCK_HOLD_MS  = 150;
static constexpr uint32_t PERFORM_NOTE_GAP_MS    = 300;

// PRE_COMPOSING timing.
static constexpr uint32_t PRECOMP_PLUCK_HOLD_MS  = 200;

// Park completion timeout (ms) — how long to wait for servos to reach park pos.
static constexpr uint32_t PARK_TIMEOUT_MS        = 3000;

class FSM {
public:
    FSM(MotorController& controller, EventDetector& detector, Composer& composer);

    void begin();
    void update(Event evt);

    State       getState()          const;
    const char* stateName(State s);
    const char* eventName(Event evt);

    // Returns true once park is complete; used by main() for clean shutdown.
    bool isParkingDone() const;

private:
    State            _state;
    bool             _homingComplete;
    MotorController& _motors;
    EventDetector&   _detector;
    Composer&        _composer;

    // --- TUNING sub-state ---
    TuningSubState _tuningSub;
    int            _tuningString;
    int            _tuningAttempt;
    int            _tuningPluckCount;
    uint32_t       _tuningSettleStart;
    float          _tuningPitchSum;
    int            _tuningReadCount;
    float          _tuningTargetHz[3];

    // --- PERFORMING sub-state ---
    int      _perfNoteIdx;
    uint32_t _perfPluckStart;
    uint32_t _perfGapStart;
    bool     _perfWaitingPluck;
    bool     _perfWaitingGap;
    bool     _perfFretDone;

    // --- PRE_COMPOSING sub-state ---
    PreCompSubState _preCompSub;
    int             _preCompNoteIdx;
    int             _preCompStringIdx;
    uint32_t        _preCompPluckStart;
    int             _preCompLastNote;    // last played note, for Markov seeding

    // --- COMPOSING sub-state ---
    bool _composingDone;

    // --- Shutdown park ---
    bool     _parkingForShutdown;
    uint32_t _parkStartMs;

    void onEnter(State s);
    void during(State s);
    void onExit(State s);
    State computeNextState(State s, Event evt);

    void _duringHoming();
    void _duringTuning();
    void _duringPerforming();
    void _duringPreComposing();
    void _duringComposing();

    void _startPark();
};