#pragma once
#include <cstdint>
#include "events.h"

// A circular buffer (ring buffer) holding pending events.
//
// TERMS:
//   "Circular buffer" — fixed array where head/tail wrap around
//                       using modulo arithmetic. No malloc, no shifting.
//   "Head"  — where the next item is written (producer)
//   "Tail"  — where the next item is read (consumer)
//   "Depth" — how many items are currently waiting

class EventQueue {
public:
    static constexpr uint8_t CAPACITY = 8;

    EventQueue();

    // Add an event to the back of the queue.
    // If full, the incoming event is DROPPED (existing events preserved).
    // Returns false if dropped.
    bool push(Event evt);

    // Remove and return the front event.
    // Returns EVT_NONE if the queue is empty.
    Event pop();

    uint8_t depth()   const;
    bool    isEmpty() const;
    bool    isFull()  const;

private:
    Event   _buf[CAPACITY];
    uint8_t _head;   // next write index
    uint8_t _tail;   // next read index
    uint8_t _count;
};