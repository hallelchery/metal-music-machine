#include <cstdio>
#include <cstring>
#include <unistd.h>   // read()
#include <termios.h>  // tcgetattr(), tcsetattr()
#include <fcntl.h>    // fcntl(), F_SETFL, O_NONBLOCK
#include "../src/fsm.h"
#include "../src/events.h"
#include "../src/event_queue.h"
#include "../src/event_detector.h"
#include "../src/motor_controller.h"
#include "../hal/hal.h"

// Put the terminal in raw, non-blocking mode so the loop can tick
// continuously without waiting for the user to press Enter.
// Returns the original terminal settings so they can be restored on exit.
static termios enableRawInput() {
    termios original;
    tcgetattr(STDIN_FILENO, &original);

    termios raw = original;
    raw.c_lflag &= ~(ICANON | ECHO); // disable line-buffering and echo
    raw.c_cc[VMIN]  = 0;             // read() returns immediately
    raw.c_cc[VTIME] = 0;             // no timeout
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    return original;
}

static void restoreInput(const termios& original) {
    tcsetattr(STDIN_FILENO, TCSANOW, &original);
}

int main() {
    hal_telemetryInit();

    MotorController motors;
    EventQueue      queue;
    EventDetector   detector(10000u);
    FSM             fsm(motors, detector);

    fsm.begin();

    printf("\nKeys: c=compose  p=pre_compose  t=tune  x=perform\n");
    printf("      s=stop     n=note_play    z=fault  i=idle_timeout\n");
    printf("      [auto] homing and tuning complete on their own.\n");
    printf("Ctrl+C to quit.\n\n");

    termios original = enableRawInput();

    while (true) {
        // --- Read one character if available; no blocking ---
        char key = 0;
        read(STDIN_FILENO, &key, 1);

        if (key) {
            switch (key) {
                case 'z': detector.injectFault();       break;
                case 'n': detector.injectNotePending();  break;
                case 's': detector.injectStop();         break;
                case 'c': detector.injectCompose();      break;
                case 'p': detector.injectPreCompose();   break;
                case 't': detector.injectTune();         break;
                case 'x': detector.injectPerform();      break;
                case 'i': detector.resetIdleTimer(0);    break;
                case 3:   // Ctrl+C
                    restoreInput(original);
                    return 0;
                default:  detector.injectWake();         break;
            }
        }

        // --- Main loop: detect → enqueue → dequeue → FSM → log ---
        uint32_t now = hal_millis();

        detector.update(fsm.getState(), now, queue);

        Event evt = queue.pop();
        fsm.update(evt);

        if (evt != Event::EVT_NONE) {
            hal_telemetryLog(now,
                             fsm.stateName(fsm.getState()),
                             fsm.eventName(evt),
                             queue.depth());
        }

        // Log motor positions each pass during motor-active states.
        State s = fsm.getState();
        if (s == State::HOMING || s == State::TUNING) {
            hal_telemetryLogMotors(now, fsm.stateName(s),
                                   motors.getStepperPosition(0),
                                   motors.getStepperPosition(1),
                                   motors.getStepperPosition(2),
                                   motors.getServoAngle(0));
        }

        // Throttle the loop to ~1ms per pass — prevents 100% CPU burn
        // and gives the sim realistic time-slice granularity.
        usleep(1000);
    }
}