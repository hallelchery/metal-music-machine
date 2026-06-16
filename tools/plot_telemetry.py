#!/usr/bin/env python3
"""
Metal Music Machine — Telemetry Visualizer
Reads telemetry.csv produced by the simulator and generates portfolio graphs.
"""

import matplotlib
matplotlib.use('Agg')  # uses non-interactive backend, no display needed
import matplotlib.pyplot as plt

import sys
import os
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

CSV_PATH = os.path.join(os.path.dirname(__file__), '..', 'telemetry.csv')
OUT_DIR  = os.path.join(os.path.dirname(__file__), '..', 'plots')

STATE_COLORS = {
    'HOMING':        '#e74c3c',
    'IDLE':          '#3498db',
    'SLEEP':         '#95a5a6',
    'PRE_COMPOSING': '#9b59b6',
    'COMPOSING':     '#f39c12',
    'TUNING':        '#1abc9c',
    'PERFORMING':    '#2ecc71',
    'ERROR':         '#e74c3c',
}


def load(path):
    df = pd.read_csv(path, dtype=str, on_bad_lines='skip')
    df['timestamp_ms'] = pd.to_numeric(df['timestamp_ms'], errors='coerce')
    for col in ['queue_depth', 's0_pos', 's1_pos', 's2_pos', 'servo0_angle',
                'tune_string', 'tune_attempt', 'measured_hz', 'target_hz',
                'tune_servo_angle', 'note_index', 'note_string']:
        df[col] = pd.to_numeric(df[col], errors='coerce')
    return df


def plot_state_timeline(df, ax):
    events = df[~df['event'].isin(['MOTORS', 'TUNE_SAMPLE', 'NOTE_SELECT'])].copy()
    if events.empty:
        return

    states = events[['timestamp_ms', 'state']].drop_duplicates()
    t_end  = df['timestamp_ms'].max()

    for i, row in states.iterrows():
        t_start = row['timestamp_ms']
        t_stop  = states.loc[states.index > i, 'timestamp_ms'].min()
        if pd.isna(t_stop):
            t_stop = t_end
        color = STATE_COLORS.get(row['state'], '#bdc3c7')
        ax.barh(0, t_stop - t_start, left=t_start, height=0.5,
                color=color, edgecolor='white', linewidth=0.5)

    patches = [mpatches.Patch(color=c, label=s) for s, c in STATE_COLORS.items()
               if s in states['state'].values]
    ax.legend(handles=patches, loc='upper right', fontsize=7, ncol=2)
    ax.set_xlabel('Time (ms)')
    ax.set_yticks([])
    ax.set_title('FSM State Timeline')


def plot_motor_positions(df, ax):
    motors = df[df['event'] == 'MOTORS'].dropna(subset=['s0_pos'])
    if motors.empty:
        return

    for col, label, color in [('s0_pos', 'String 0', '#e74c3c'),
                                ('s1_pos', 'String 1', '#3498db'),
                                ('s2_pos', 'String 2', '#2ecc71')]:
        ax.plot(motors['timestamp_ms'], motors[col], label=label,
                color=color, linewidth=0.8, alpha=0.85)

    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Fret Stepper Position (steps)')
    ax.set_title('Fret Stepper Positions Over Time')
    ax.legend(fontsize=8)


def plot_tuning_convergence(df, ax):
    tuning = df[df['event'] == 'TUNE_SAMPLE'].dropna(subset=['measured_hz'])
    if tuning.empty:
        ax.set_title('Tuning Convergence (no data)')
        return

    for string_id in tuning['tune_string'].dropna().unique():
        sub = tuning[tuning['tune_string'] == string_id].sort_values('tune_attempt')
        ax.plot(sub['tune_attempt'], sub['measured_hz'],
                marker='o', label=f'String {int(string_id)} measured')
        if not sub['target_hz'].isna().all():
            target = sub['target_hz'].iloc[0]
            ax.axhline(target, linestyle='--', alpha=0.5,
                       label=f'String {int(string_id)} target ({target:.1f} Hz)')

    ax.set_xlabel('Attempt')
    ax.set_ylabel('Frequency (Hz)')
    ax.set_title('Tuning Convergence per String')
    ax.legend(fontsize=7)


def plot_queue_depth(df, ax):
    events = df[~df['event'].isin(['MOTORS', 'TUNE_SAMPLE', 'NOTE_SELECT'])].dropna(subset=['queue_depth'])
    if events.empty:
        return

    ax.fill_between(events['timestamp_ms'], events['queue_depth'],
                    step='post', alpha=0.6, color='#e67e22')
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Queue Depth (events)')
    ax.set_title('Event Queue Depth Over Time')
    ax.set_ylim(bottom=0)


def plot_note_distribution(df, ax):
    notes = df[df['event'] == 'NOTE_SELECT'].dropna(subset=['note_index'])
    if notes.empty:
        ax.set_title('Note Distribution (no PRE_COMPOSING data)')
        return

    counts = notes['note_index'].value_counts().sort_index()
    note_names = ['C', 'C#', 'D', 'D#', 'E', 'F',
                  'F#', 'G', 'G#', 'A', 'A#', 'B']
    labels = [note_names[int(i)] if 0 <= int(i) < 12 else str(int(i))
              for i in counts.index]
    ax.bar(labels, counts.values, color='#9b59b6', alpha=0.85)
    ax.set_xlabel('Note')
    ax.set_ylabel('Times Selected')
    ax.set_title('PRE_COMPOSING Note Selections')


def main():
    if not os.path.exists(CSV_PATH):
        print(f'Error: telemetry.csv not found at {CSV_PATH}')
        sys.exit(1)

    os.makedirs(OUT_DIR, exist_ok=True)
    df = load(CSV_PATH)

    fig, axes = plt.subplots(5, 1, figsize=(14, 18))
    fig.suptitle('Metal Music Machine — Telemetry Report', fontsize=14, fontweight='bold')

    plot_state_timeline(df, axes[0])
    plot_motor_positions(df, axes[1])
    plot_tuning_convergence(df, axes[2])
    plot_queue_depth(df, axes[3])
    plot_note_distribution(df, axes[4])

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    out_path = os.path.join(OUT_DIR, 'telemetry_report.png')
    plt.savefig(out_path, dpi=150)
    print(f'Saved: {out_path}')
    plt.savefig("output_plot.png")  # saves plot as a file
    print("Plot saved as output_plot.png")


if __name__ == '__main__':
    main()