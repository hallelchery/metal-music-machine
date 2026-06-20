#!/usr/bin/env python3
"""
midi_from_telemetry.py — Convert MMM telemetry note log to a MIDI file.

Reads telemetry.csv, extracts rows where note_index is populated,
and writes a MIDI file with one note per logged event.

Usage:
    python3 tools/midi_from_telemetry.py [telemetry.csv] [output.mid]
"""

import sys
import csv
from midiutil import MIDIFile

# MIDI pitch of each open string (G3, B3, D4).
STRING_BASE_PITCH = [55, 59, 62]

TEMPO       = 120   # BPM — arbitrary, just for audition
BEAT_DUR    = 0.5   # duration of each note in beats
BEAT_GAP    = 0.5   # silence between notes in beats
CHANNEL     = 0
VOLUME      = 100
TRACK       = 0

def load_notes(csv_path: str) -> list[tuple[int, int]]:
    notes = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            row = {k.strip(): (v.strip() if v is not None else "") for k, v in row.items()}
            ni = row.get("note_index", "")
            ns = row.get("note_string", "")
            if ni and ns:
                notes.append((int(ns), int(ni)))
    return notes

def notes_to_midi(notes: list[tuple[int, int]], out_path: str) -> None:
    """Write notes to a MIDI file."""
    midi = MIDIFile(numTracks=1)
    midi.addTempo(TRACK, 0, TEMPO)

    beat = 0.0
    for string_id, note_index in notes:
        if string_id < 0 or string_id >= len(STRING_BASE_PITCH):
            print(f"  [WARN] Skipping invalid string_id {string_id}")
            continue
        pitch = STRING_BASE_PITCH[string_id] + note_index
        if pitch < 0 or pitch > 127:
            print(f"  [WARN] Skipping out-of-range MIDI pitch {pitch}")
            continue
        midi.addNote(TRACK, CHANNEL, pitch, beat, BEAT_DUR, VOLUME)
        beat += BEAT_DUR + BEAT_GAP

    with open(out_path, "wb") as f:
        midi.writeFile(f)

    print(f"Wrote {len(notes)} notes to {out_path}")


def main() -> None:
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "telemetry.csv"
    out_path = sys.argv[2] if len(sys.argv) > 2 else "output.mid"

    notes = load_notes(csv_path)
    if not notes:
        print(f"No note events found in {csv_path}. Run the simulator first.")
        sys.exit(1)

    notes_to_midi(notes, out_path)


if __name__ == "__main__":
    main()