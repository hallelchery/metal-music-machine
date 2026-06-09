#include <cstdio>
#include <cstring>
#include "../src/fsm.h"
#include "../src/events.h"
#include "../src/event_queue.h"
#include "../src/event_detector.h"
#include "../hal/hal.h"

int main() {
    hal_telemetryInit();

    FSM           fsm;
    EventQueue    queue;
    // 10-second idle timeout. Lowercase 'u' suffix = unsigned literal,
    // matching the uint32_t parameter type exactly.
    EventDetector detector(10000u);

    fsm.begin();

    printf("\nKeys: h=homing_done  c=compose  p=pre_compose  t=tune\n");
    printf("      x=perform      s=stop     n=note_play    z=fault  i=idle_timeout\n");
    printf("Press Enter after each key. Ctrl+C to quit.\n\n");

    char input[16];

    while (true) {
        if (fgets(input, sizeof(input), stdin) == nullptr) break;
        if (input[0] == '\n') continue;

        // --- Translate keypress into a detector signal ---
        // We no longer build an Event directly. Instead we tell the
        // detector what raw input just arrived, and let it handle
        // debouncing, priority, and queueing.
        char key = input[0];
        switch (key) {
            case 'h': detector.injectDone();                          break;
            case 'z': detector.injectFault();                         break;
            case 'n': detector.injectNotePending();                   break;
            case 's': detector.injectStop();                          break;
            case 'c': detector.injectCompose();                       break;
            case 'p': detector.injectPreCompose();                    break;
            case 't': detector.injectTune();                          break;
            case 'x': detector.injectPerform();                       break;
            case 'i': // force idle timeout for testing
                detector.resetIdleTimer(0); break;
            default:  // any other key = wake signal (all buttons)
                detector.injectWake(); break;
        }

        // --- The main loop: detect → enqueue → dequeue → FSM → log ---
        // This is the heartbeat pattern every iteration builds on.
        uint32_t now = hal_millis();

        // 1. Detector scans inputs, debounces, pushes to queue
        detector.update(fsm.getState(), now, queue);

        // 2. FSM drains one event per pass
        Event evt = queue.pop();
        fsm.update(evt);

        // 3. Telemetry: what happened this pass, and how loaded was the queue?
        hal_telemetryLog(now,
                         fsm.stateName(fsm.getState()),
                         fsm.eventName(evt),
                         queue.depth());

        printf("\n");
    }

    return 0;
}