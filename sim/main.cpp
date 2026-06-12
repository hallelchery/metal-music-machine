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
    FSM             fsm(motors, detector);

    fsm.begin();

    printf("\n=== Metal Music Machine Simulator ===\n");
    printf("Global:  s=stop  z=fault  i=force_idle_timeout\n");
    printf("IDLE:    c=compose  p=pre_compose  t=tune  x=perform\n");
    printf("PRE_COMPOSING: n=play_note  x=next_note  t=next_string  c=go_compose\n");
    printf("SLEEP:   [any key]=wake\n");
    printf("Ctrl+C to quit.\n\n");

    termios original = enableRawInput();

    while (true) {
        char key = 0;
        read(STDIN_FILENO, &key, 1);

        if (key) {
            switch (key) {
                case 'z': detector.injectFault();      break;
                case 'n': detector.injectNotePending(); break;
                case 's': detector.injectStop();        break;
                case 'c': detector.injectCompose();     break;
                case 'p': detector.injectPreCompose();  break;
                case 't': detector.injectTune();        break;
                case 'x': detector.injectPerform();     break;
                case 'i': detector.resetIdleTimer(0);   break;
                case 3:
                    restoreInput(original);
                    return 0;
                default:  detector.injectWake();        break;
            }
        }

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

        State s = fsm.getState();
        if (s == State::HOMING || s == State::TUNING || s == State::PERFORMING) {
            hal_telemetryLogMotors(now, fsm.stateName(s),
                                   motors.getStepperPosition(0),
                                   motors.getStepperPosition(1),
                                   motors.getStepperPosition(2),
                                   motors.getServoAngle(0));
        }

        usleep(1000);
    }
}