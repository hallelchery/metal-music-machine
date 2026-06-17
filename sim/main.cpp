#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "../src/fsm.h"
#include "../src/events.h"
#include "../src/event_queue.h"
#include "../src/event_detector.h"
#include "../src/motor_controller.h"
#include "../src/composer.h"
#include "../hal/hal.h"

static termios enableRawInput() {
    termios original;
    tcgetattr(STDIN_FILENO, &original);
    termios raw = original;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;
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
    Composer        composer;
    FSM             fsm(motors, detector, composer);

    fsm.begin();

    printf("\n=== Metal Music Machine Simulator ===\n");
    printf("--- Global keys (any state) ---\n");
    printf("  s = stop/abort    z = inject fault    q = quit (parks motors)\n");
    printf("--- IDLE ---\n");
    printf("  c = compose    p = pre-compose    t = tune    x = perform\n");
    printf("--- PRE_COMPOSING ---\n");
    printf("  n = play note    x = next note    t = next string    c = go to compose\n");
    printf("--- SLEEP ---\n");
    printf("  [any key except s/z/q] = wake\n");
    printf("--- Debug ---\n");
    printf("  i = force idle timeout (IDLE/PRE_COMPOSING/COMPOSING only)\n\n");

    termios original = enableRawInput();
    bool quitting = false;

    while (true) {
        char key = 0;
        read(STDIN_FILENO, &key, 1);

        State currentState = fsm.getState();

        if (!quitting && key) {
            switch (key) {
                case 'z': detector.injectFault();        break;
                case 'n': detector.injectNotePending();  break;
                case 's': detector.injectStop();         break;
                case 'c':
                    if (currentState == State::SLEEP)    detector.injectWake();
                    else                                 detector.injectCompose();
                    break;
                case 'p':
                    if (currentState == State::SLEEP)    detector.injectWake();
                    else                                 detector.injectPreCompose();
                    break;
                case 't':
                    if (currentState == State::SLEEP)    detector.injectWake();
                    else                                 detector.injectTune();
                    break;
                case 'x':
                    if (currentState == State::SLEEP)    detector.injectWake();
                    else                                 detector.injectPerform();
                    break;
                case 'i': detector.injectIdleTimeout();  break;
                case 'q':
                    printf("\n[SIM] Quit requested — parking all actuators...\n");
                    quitting = true;
                    motors.parkAll();
                    break;
                case 3:
                    restoreInput(original);
                    return 0;
                default:
                    detector.injectWake();
                    break;
            }
        }

        uint32_t now = hal_millis();
        motors.tick(now);

        if (!quitting) {
            detector.update(fsm.getState(), now, queue);
            Event evt = queue.pop();
            
            const char* stateBeforeUpdate = fsm.stateName(fsm.getState());
            
            fsm.update(evt);

            if (evt != Event::EVT_NONE) {
                hal_telemetryLog(now,
                                 stateBeforeUpdate,
                                 fsm.eventName(evt),
                                 queue.depth());
            }

            State s = fsm.getState();
            if (s == State::HOMING || s == State::TUNING || s == State::PERFORMING) {
                hal_telemetryLogMotors(now, fsm.stateName(s),
                                       motors.getStepperPosition(0),
                                       motors.getStepperPosition(1),
                                       motors.getStepperPosition(2),
                                       motors.getServoAngle(0));
            }
        } else {
            if (motors.isParkComplete()) {
                printf("[SIM] Park complete. Exiting cleanly.\n");
                restoreInput(original);
                return 0;
            }
            // Timeout safety: don't hang forever if park fails.
            static uint32_t parkStartMs = now;
            if ((now - parkStartMs) > 3000) {
                printf("[SIM] Park timeout. Exiting.\n");
                restoreInput(original);
                return 0;
            }
        }

        usleep(1000);
    }
}