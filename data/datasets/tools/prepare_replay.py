#!/usr/bin/env python3
"""Export a LABS LeRobot episode into a small, video-free replay bundle."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def prepare(dataset, output, episode=0):
    import pyarrow.parquet as pq

    info = json.loads((dataset / 'meta/info.json').read_text())
    modalities = json.loads((dataset / 'meta/modality.json').read_text())
    expected = {'left_arm_position': (0, 7), 'left_gripper_position': (7, 8),
                'right_arm_position': (8, 15), 'right_gripper_position': (15, 16)}
    for name, bounds in expected.items():
        value = modalities['action'][name]
        if (value['start'], value['end']) != bounds:
            raise ValueError(f'Unsupported action layout: {name}: {value}')
    if info['features']['action']['shape'] != [16]:
        raise ValueError('Expected LABS 16-dimensional actions')
    for side, bounds in [('left', (0, 7)), ('right', (28, 35))]:
        value = modalities['state'][f'{side}_arm_position']
        if (value['start'], value['end']) != bounds:
            raise ValueError(f'Unsupported state layout for {side}')
    source = dataset / info['data_path'].format(
        episode_chunk=episode // info['chunks_size'], episode_index=episode)
    table = pq.read_table(source, columns=['action', 'observation.state',
                                         'timestamp', 'frame_index', 'episode_index'])
    actions = np.array(table['action'].to_pylist(), dtype=np.float64)
    states = np.array(table['observation.state'].to_pylist(), dtype=np.float64)
    times = np.array(table['timestamp'].to_pylist(), dtype=np.float64).reshape(-1)
    frames = np.array(table['frame_index'].to_pylist())
    episodes = np.array(table['episode_index'].to_pylist())
    if (len(times) < 2 or actions.shape != (len(times), 16)
            or states.shape != (len(times), 56)
            or not np.array_equal(frames, np.arange(len(times)))
            or not np.all(episodes == episode)):
        raise ValueError('Invalid episode dimensions or indices')
    if not all(np.isfinite(a).all() for a in (times, actions, states)):
        raise ValueError('Non-finite dataset values')
    fps = float(info['fps'])
    if fps <= 0 or not np.allclose(np.diff(times), 1 / fps, atol=1e-5, rtol=0):
        raise ValueError('Nonuniform timestamps or invalid FPS')
    if np.any((actions[:, [7, 15]] < 0) | (actions[:, [7, 15]] > 1)):
        raise ValueError('Gripper commands must be open fractions in [0, 1]')
    q = states[:, np.r_[0:7, 28:35]]
    arm_actions = actions[:, np.r_[0:7, 8:15]]
    output.mkdir(parents=True, exist_ok=True)
    bundle = output / 'episode.npz'
    np.savez_compressed(bundle, timestamps=times - times[0], actions=actions,
                        measured_joints=q, measured_grippers=states[:, [14, 42]], fps=np.array(fps))
    report = {
        'dataset': str(dataset.resolve()), 'episode': episode, 'frames': len(times),
        'fps': fps, 'duration_seconds': len(times) / fps,
        'source_parquet': str(source),
        'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'bundle_sha256': hashlib.sha256(bundle.read_bytes()).hexdigest(),
        'action_layout': 'left joints[0:7], left open[7], right joints[8:15], right open[15]',
        'initial_measured_joints': q[0].tolist(),
        'initial_action_joints': arm_actions[0].tolist(),
        'initial_gripper_commands': actions[0, [7, 15]].tolist(),
        'initial_measured_grippers_rad': states[0, [14, 42]].tolist(),
        'max_initial_action_state_difference_rad': float(np.abs(arm_actions[0] - q[0]).max()),
        'peak_recorded_command_speed_rad_s':
            (np.abs(np.diff(arm_actions, axis=0)) / np.diff(times)[:, None]).max(axis=0).tolist(),
        'recorded_command_measured_max_difference_rad':
            np.abs(arm_actions - q).max(axis=0).tolist(),
        'base_and_spine': 'Not recorded in this dataset; restore their recording configuration manually.',
    }
    (output / 'manifest.json').write_text(json.dumps(report, indent=2) + '\n')
    (output / 'home_pose.json').write_text(json.dumps(
        {'left': q[0, :7].tolist(), 'right': q[0, 7:].tolist()}, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--episode', type=int, default=0)
    args = parser.parse_args()
    prepare(args.dataset, args.output, args.episode)
