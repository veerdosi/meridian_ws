#!/usr/bin/env python3
"""Read a rosbag2 bag, extract Fz trace for an episode, render a plot,
and optionally call the Anthropic VLM for failure annotation.

Usage:
    python3 annotate_episode.py --bag /root/meridian_ws/bags/20240101_120000 --episode 2

If ANTHROPIC_API_KEY is not set, the script saves the plot image and exits cleanly.
"""

import argparse
import base64
import json
import os
import sqlite3
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def find_db3(bag_path: str) -> str:
    bag_dir = Path(bag_path)
    db3_files = list(bag_dir.glob('*.db3'))
    if not db3_files:
        raise FileNotFoundError(f'No .db3 file found in {bag_dir}')
    return str(db3_files[0])


def get_topic_ids(conn: sqlite3.Connection, topic_names: list) -> dict:
    cursor = conn.execute('SELECT id, name FROM topics')
    return {row[1]: row[0] for row in cursor if row[1] in topic_names}


def read_messages(conn: sqlite3.Connection, topic_id: int) -> list:
    cursor = conn.execute(
        'SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
        (topic_id,),
    )
    return cursor.fetchall()


def _deser_compliance_state(data: bytes):
    import rclpy.serialization
    from meridian_control.msg import ComplianceState
    return rclpy.serialization.deserialize_message(data, ComplianceState)


def _deser_wrench(data: bytes):
    import rclpy.serialization
    from geometry_msgs.msg import WrenchStamped
    return rclpy.serialization.deserialize_message(data, WrenchStamped)


def _deser_outcome(data: bytes):
    import rclpy.serialization
    from meridian_control.msg import ExecutionOutcome
    return rclpy.serialization.deserialize_message(data, ExecutionOutcome)


def find_episode_boundaries(conn: sqlite3.Connection, state_topic_id: int, episode_num: int):
    """Return (contact_start_ns, contact_end_ns, final_state) for episode N (1-indexed).

    Scans state messages for CONTACT_ACTIVE -> SEATED/FAILURE transitions.
    Each such transition represents one episode's contact phase.
    Returns None if episode_num exceeds the number of episodes found.
    """
    rows = read_messages(conn, state_topic_id)
    episodes = []
    prev_state = None
    contact_start = None

    for ts, data in rows:
        try:
            msg = _deser_compliance_state(data)
        except Exception:
            continue
        state = msg.state

        if state == 'CONTACT_ACTIVE' and prev_state != 'CONTACT_ACTIVE':
            contact_start = ts

        if (contact_start is not None
                and prev_state == 'CONTACT_ACTIVE'
                and state in ('SEATED', 'FAILURE')):
            episodes.append((contact_start, ts, state))
            contact_start = None

        prev_state = state

    if episode_num < 1 or episode_num > len(episodes):
        return None
    return episodes[episode_num - 1]


def extract_fz_trace(
    conn: sqlite3.Connection, ft_topic_id: int, start_ns: int, end_ns: int
) -> tuple:
    """Return (times_sec_from_contact, fz_values) arrays."""
    cursor = conn.execute(
        'SELECT timestamp, data FROM messages'
        ' WHERE topic_id=? AND timestamp >= ? AND timestamp <= ? ORDER BY timestamp',
        (ft_topic_id, start_ns, end_ns),
    )
    rows = cursor.fetchall()

    times, fz_values = [], []
    t0 = None
    for ts, data in rows:
        try:
            msg = _deser_wrench(data)
        except Exception:
            continue
        if t0 is None:
            t0 = ts
        times.append((ts - t0) * 1e-9)
        fz_values.append(msg.wrench.force.z)

    return np.array(times), np.array(fz_values)


def get_episode_outcome(conn: sqlite3.Connection, outcome_topic_id: int, episode_num: int):
    """Return the Nth ExecutionOutcome message (1-indexed), or None."""
    rows = read_messages(conn, outcome_topic_id)
    if episode_num < 1 or episode_num > len(rows):
        return None
    try:
        return _deser_outcome(rows[episode_num - 1][1])
    except Exception:
        return None


def render_plot(
    times: np.ndarray,
    fz_values: np.ndarray,
    end_time_sec: float,
    outcome: str,
    episode_num: int,
    output_path: str,
) -> None:
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(times, fz_values, color='#2196F3', linewidth=1.5, label='Fz filtered')
    if end_time_sec is not None and len(times) > 0 and 0 <= end_time_sec <= times[-1]:
        ax.axvline(x=end_time_sec, color='red', linestyle='--', linewidth=2,
                   label='Abort/Seat event')
    ax.set_xlabel('Time from contact (s)')
    ax.set_ylabel('Force (N)')
    ax.set_title(f'Episode {episode_num} — {outcome}')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def call_vlm(image_path: str, outcome: str, outcome_msg) -> dict | None:
    api_key = os.environ.get('ANTHROPIC_API_KEY')
    if not api_key:
        print('[MERIDIAN] API key not set — skipping VLM call')
        return None

    import anthropic

    with open(image_path, 'rb') as f:
        image_b64 = base64.standard_b64encode(f.read()).decode('utf-8')

    failure_mode = outcome_msg.failure_mode if outcome_msg else 'unknown'
    peak_force = outcome_msg.peak_force if outcome_msg else 0.0
    insertion_dist = outcome_msg.insertion_distance if outcome_msg else 0.0
    duration = outcome_msg.duration_sec if outcome_msg else 0.0
    event_label = 'seating event' if outcome == 'SUCCESS' else 'abort event'

    prompt = (
        'You are analyzing a force/torque trace from a robotic USB-C connector insertion attempt.\n\n'
        f'Task class: connector insertion\n'
        f'Outcome: {outcome}\n'
        f'Failure mode (if any): {failure_mode}\n'
        f'Peak force: {peak_force:.2f} N\n'
        f'Insertion distance: {insertion_dist:.4f} m\n'
        f'Duration: {duration:.2f} s\n\n'
        'The attached image shows filtered Fz (insertion-axis force) over time during the contact phase.\n'
        f'The vertical red line marks the {event_label}.\n\n'
        'Respond ONLY with a JSON object — no preamble, no markdown backticks:\n'
        '{\n'
        '  "failure_hypothesis": "string",\n'
        '  "failure_category": "misalignment | over_force | cross_thread | timeout | slip | incomplete_seat | unknown",\n'
        '  "confidence": 0.0,\n'
        '  "key_evidence": ["string"],\n'
        '  "corrective_action": "string",\n'
        '  "requires_human_review": false\n'
        '}'
    )

    client = anthropic.Anthropic(api_key=api_key)
    response = client.messages.create(
        model='claude-sonnet-4-20250514',
        max_tokens=1024,
        messages=[{
            'role': 'user',
            'content': [
                {
                    'type': 'image',
                    'source': {
                        'type': 'base64',
                        'media_type': 'image/png',
                        'data': image_b64,
                    },
                },
                {'type': 'text', 'text': prompt},
            ],
        }],
    )

    try:
        return json.loads(response.content[0].text)
    except (json.JSONDecodeError, IndexError, AttributeError) as e:
        print(f'[MERIDIAN] Failed to parse VLM response: {e}')
        return None


def main() -> None:
    parser = argparse.ArgumentParser(description='Annotate a rosbag2 episode with VLM')
    parser.add_argument('--bag', required=True, help='Path to bag directory')
    parser.add_argument('--episode', type=int, required=True, help='Episode number (1-indexed)')
    args = parser.parse_args()

    import rclpy
    rclpy.init(args=[])

    exit_code = 0
    try:
        db3_path = find_db3(args.bag)
        conn = sqlite3.connect(db3_path)

        topics_needed = [
            '/ft_sensor/filtered',
            '/compliance_controller/state',
            '/execution_outcome',
        ]
        topic_ids = get_topic_ids(conn, topics_needed)

        missing = [t for t in topics_needed if t not in topic_ids]
        if missing:
            print(f'[MERIDIAN] Missing topics in bag: {missing}')
            conn.close()
            exit_code = 1
        else:
            boundaries = find_episode_boundaries(
                conn, topic_ids['/compliance_controller/state'], args.episode
            )
            if boundaries is None:
                print(f'[MERIDIAN] Episode {args.episode} not found in bag (bag may not have enough episodes)')
                conn.close()
                exit_code = 1
            else:
                contact_start_ns, contact_end_ns, final_state = boundaries
                outcome = 'SUCCESS' if final_state == 'SEATED' else 'FAILURE'

                times, fz_values = extract_fz_trace(
                    conn, topic_ids['/ft_sensor/filtered'], contact_start_ns, contact_end_ns
                )

                if len(times) == 0:
                    print(f'[MERIDIAN] No Fz data found for episode {args.episode}')
                    conn.close()
                    exit_code = 1
                else:
                    end_time_sec = (contact_end_ns - contact_start_ns) * 1e-9

                    outcome_msg = get_episode_outcome(conn, topic_ids['/execution_outcome'], args.episode)
                    conn.close()

                    plot_path = f'episode_{args.episode}_ft_trace.png'
                    render_plot(times, fz_values, end_time_sec, outcome, args.episode, plot_path)
                    print(f'[MERIDIAN] Saved {plot_path}')

                    print(f'[MERIDIAN] Episode {args.episode}: {outcome}'
                          + (f' peak={outcome_msg.peak_force:.2f}N' if outcome_msg else ''))

                    annotation = call_vlm(plot_path, outcome, outcome_msg)

                    if annotation is not None:
                        print('\n[MERIDIAN] VLM Annotation:')
                        print(json.dumps(annotation, indent=2))

                        output_data = {
                            'episode': args.episode,
                            'outcome': outcome,
                            'peak_force': float(outcome_msg.peak_force) if outcome_msg else None,
                            'insertion_distance': float(outcome_msg.insertion_distance) if outcome_msg else None,
                            'duration_sec': float(outcome_msg.duration_sec) if outcome_msg else None,
                            'failure_mode': outcome_msg.failure_mode if outcome_msg else None,
                            'annotation': annotation,
                        }

                        json_path = f'episode_{args.episode}_annotation.json'
                        with open(json_path, 'w') as f:
                            json.dump(output_data, f, indent=2)
                        print(f'[MERIDIAN] Saved {json_path}')
    finally:
        rclpy.shutdown()

    if exit_code != 0:
        sys.exit(exit_code)


if __name__ == '__main__':
    main()
