#include <cstdio>
#include <cstring>
#include "../src/fsm.h"
#include "../src/events.h"
#include "../hal/hal.h"

// Translates a keypress character into an FSM Event.
Event keyToEvent(char key) {
    switch (key) {
        case 'h': return Event::EVT_DONE;           // simulate homing complete
        case 'c': return Event::EVT_BTN_COMPOSE;
        case 'p': return Event::EVT_BTN_PRE_COMPOSE;
        case 't': return Event::EVT_BTN_TUNE;
        case 'x': return Event::EVT_BTN_PERFORM;
        case 's': return Event::EVT_BTN_STOP;
        case 'n': return Event::EVT_NOTE_PLAY;
        case 'z': return Event::EVT_FAULT;
        case 'i': return Event::EVT_IDLE_TIMEOUT;   // manually trigger timeout for testing
        default:  return Event::EVT_BTN_WAKE;       // any other key = wake from sleep
    }
}

int main() {
    // One-time setup
    hal_telemetryInit();

    FSM fsm;
    fsm.begin();

    printf("\nKeys: h=homing_done  c=compose  p=pre_compose  t=tune\n");
    printf("      x=perform      s=stop     n=note_play    z=fault  i=idle_timeout\n");
    printf("Press Enter after each key. Ctrl+C to quit.\n\n");

    char input[16];

    // The main loop — runs forever until Ctrl+C
    while (true) {
        // Read a line of input from the terminal
        if (fgets(input, sizeof(input), stdin) == nullptr) break;

        // fgets includes the newline; ignore empty presses
        if (input[0] == '\n') continue;

        char key = input[0];
        Event evt = keyToEvent(key);

        // Hand the event to the FSM
        fsm.update(evt);

        printf("\n"); // Blank line for readability between events
    }

    return 0;
}