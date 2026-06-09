#include "event_queue.h"

EventQueue::EventQueue()
    : _head(0), _tail(0), _count(0)
{
    for (uint8_t i = 0; i < CAPACITY; i++) {
        _buf[i] = Event::EVT_NONE;
    }
}

bool EventQueue::push(Event evt) {
    if (isFull()) {
        // Overflow policy: drop the incoming event.
        // Events already queued represent work the FSM hasn't seen yet —
        // they're more important than a new arrival.
        return false;
    }
    _buf[_head] = evt;
    // Modulo wraps _head back to 0 when it reaches CAPACITY.
    // This is what makes the buffer "circular."
    _head = (_head + 1) % CAPACITY;
    _count++;
    return true;
}

Event EventQueue::pop() {
    if (isEmpty()) {
        return Event::EVT_NONE;
    }
    Event evt = _buf[_tail];
    _tail = (_tail + 1) % CAPACITY;
    _count--;
    return evt;
}

uint8_t EventQueue::depth()   const { return _count; }
bool    EventQueue::isEmpty() const { return _count == 0; }
bool    EventQueue::isFull()  const { return _count == CAPACITY; }