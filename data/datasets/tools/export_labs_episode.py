#!/usr/bin/env python3
"""Export a completed labs processed episode using LeRobot 0.3.3 (v2.1).

Run in the existing labs/dataset-builder container; see the adjacent shell script.
All input mounts are read-only. The destination must not already exist.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import shutil
from fractions import Fraction
from datetime import datetime, timezone

import av
import datasets
import numpy as np
from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory as ProtoDecoder
from mcap_ros2.decoder import DecoderFactory as RosDecoder
from lerobot.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
from lerobot.datasets.compute_stats import compute_episode_stats
from lerobot.datasets.utils import get_hf_features_from_features


def log(message):
    print(f"{datetime.now(timezone.utc).isoformat()} {message}", flush=True)


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def specifications():
    state, action = {}, {}
    for side in ("left", "right"):
        root = f"/{side}/franka_robot_state_broadcaster"
        state[f"{root}/measured_joint_states"] = (
            [f"{side}_joint{i}_{field}" for field in ("position", "velocity") for i in range(1, 8)],
            lambda m: list(m.position) + list(m.velocity),
        )
        state[f"/{side}/follower/gripper/joint_states"] = (
            [f"{side}_gripper_position"], lambda m: list(m.position),
        )
        state[f"{root}/external_joint_torques"] = (
            [f"{side}_joint{i}_external_torque" for i in range(1, 8)], lambda m: list(m.effort),
        )
        state[f"{root}/external_wrench_in_stiffness_frame"] = (
            [f"{side}_{field}_{axis}" for field in ("force", "torque") for axis in "xyz"],
            lambda m: [getattr(getattr(m.wrench, field), axis) for field in ("force", "torque") for axis in "xyz"],
        )
        action[f"/{side}/follower/gello/joint_states"] = (
            [f"{side}_joint{i}_target_position" for i in range(1, 8)], lambda m: list(m.position),
        )
        action[f"/{side}/follower/gripper/gripper_client/target_gripper_width_percent"] = (
            [f"{side}_gripper_target_width_fraction"], lambda m: [m.data],
        )
    return state, action


def nearest_indices(source, target):
    right = np.searchsorted(source, target).clip(0, len(source) - 1)
    left = (right - 1).clip(0)
    return np.where(abs(source[left] - target) <= abs(source[right] - target), left, right)


def export_video(payload, times, grid, path, fps):
    """Decode once, pick frames by original MCAP time, and encode a regular timeline."""
    chosen = nearest_indices(times, grid)
    path.parent.mkdir(parents=True, exist_ok=True)
    count, decoded_count = 0, 0
    sums = np.zeros(3, dtype=np.float64)
    squares = sums.copy()
    minimum, maximum = np.full(3, 255.0), np.zeros(3)
    pixels = 0
    with av.open(io.BytesIO(payload)) as source, av.open(str(path), "w") as output:
        original = source.streams.video[0]
        original.thread_count = 2
        stream = output.add_stream("libx264", rate=fps)
        stream.width, stream.height = original.width, original.height
        stream.pix_fmt = "yuv420p"
        stream.codec_context.thread_count = 2
        stream.options = {"crf": "18", "preset": "veryfast", "g": "2", "bf": "0"}
        shape = (original.height, original.width, 3)
        for source_index, decoded in enumerate(source.decode(video=0)):
            decoded_count += 1
            if count == len(grid) or source_index != chosen[count]:
                continue
            rgb = decoded.to_ndarray(format="rgb24")
            step = max(1, max(shape[:2]) // 128)
            sample = rgb[::step, ::step].reshape(-1, 3).astype(np.float64)
            while count < len(grid) and source_index == chosen[count]:
                frame = av.VideoFrame.from_ndarray(rgb, format="rgb24")
                frame.pts, frame.time_base = count, Fraction(1, fps)
                for packet in stream.encode(frame):
                    output.mux(packet)
                sums += sample.sum(axis=0)
                squares += np.square(sample).sum(axis=0)
                minimum = np.minimum(minimum, sample.min(axis=0))
                maximum = np.maximum(maximum, sample.max(axis=0))
                pixels += len(sample)
                count += 1
                if count % 500 == 0:
                    log(f"{path.parent.name}: encoded {count}/{len(grid)} frames")
        for packet in stream.encode():
            output.mux(packet)
    assert decoded_count == len(times), (decoded_count, len(times))
    assert count == len(grid), (count, len(grid))
    mean = sums / pixels
    std = np.sqrt(np.maximum(squares / pixels - mean**2, 0))
    stats = {key: (value / 255).reshape(3, 1, 1) for key, value in
             {"min": minimum, "max": maximum, "mean": mean, "std": std}.items()}
    stats["count"] = np.array([count])
    gaps = abs(times[chosen] - grid) / 1e6
    return shape, stats, chosen, {"source_frames": decoded_count, "output_frames": count,
        "max_alignment_error_ms": float(gaps.max()), "mean_alignment_error_ms": float(gaps.mean())}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-episode", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--modality", type=Path, required=True)
    parser.add_argument("--fps", type=int, default=20)
    args = parser.parse_args()
    assert args.fps > 0 and 1_000_000_000 % args.fps == 0
    assert not args.output.exists(), f"Output already exists: {args.output}"
    assert (args.input_episode / ".processing_complete").exists()
    source_file = args.input_episode / "mcap/mcap_0.mcap"
    episode = json.loads((args.input_episode / "episode_metadata.json").read_text())
    task = episode["task_description"]
    state_specs, action_specs = specifications()
    specs = {**state_specs, **action_specs}
    telemetry = {topic: {"times": [], "values": []} for topic in specs}
    videos = {}
    with source_file.open("rb") as handle:
        reader = make_reader(handle, decoder_factories=[RosDecoder(), ProtoDecoder()])
        metadata = {m.name: json.loads(m.metadata["data"]) for m in reader.iter_metadata() if "data" in m.metadata}
        mapping = metadata["conversion_info"]["topic_mapping"]
        camera_topics = {"head": mapping["/head_camera/zed_node/rgb/color/rect/image"],
            "wrist_left": mapping["/wrist_camera_left/camera/color/image_raw"],
            "wrist_right": mapping["/wrist_camera_right/camera/color/image_raw"]}
        selected_topics = list(specs) + list(camera_topics.values())
        log("Reading telemetry and compressed video from processed MCAP")
        for number, (schema, channel, message, decoded) in enumerate(reader.iter_decoded_messages(topics=selected_topics)):
            topic = channel.topic
            if topic in specs:
                values = specs[topic][1](decoded)
                assert len(values) == len(specs[topic][0]), (topic, len(values))
                telemetry[topic]["times"].append(message.log_time)
                telemetry[topic]["values"].append(values)
            else:
                assert topic not in videos, f"Expected one embedded video per camera: {topic}"
                videos[topic] = bytes(decoded.data)
            if number and number % 200000 == 0:
                log(f"Read {number} messages")
    bounds = []
    topic_report = {}
    for topic, data in telemetry.items():
        times = np.array(data["times"], dtype=np.int64)
        values = np.array(data["values"], dtype=np.float64)
        assert len(times) > 1 and np.isfinite(values).all(), topic
        assert np.all(np.diff(times) >= 0), topic
        unique = np.r_[np.diff(times) != 0, True]  # Keep last message at identical log_time.
        data["times"], data["values"] = times[unique], values[unique]
        bounds.append((int(times[0]), int(times[-1])))
        topic_report[topic] = {"messages": len(times), "dimensions": values.shape[1],
            "max_gap_ms": float(np.diff(times).max() / 1e6)}
    camera_times = {}
    for alias, topic in camera_topics.items():
        assert topic in videos, topic
        times = np.array(metadata["conversion_info"]["frame_boundaries"][topic]["frame_timestamps"], dtype=np.int64)
        assert np.all(np.diff(times) > 0)
        camera_times[alias] = times
        bounds.append((int(times[0]), int(times[-1])))
    start, end = max(x[0] for x in bounds), min(x[1] for x in bounds)
    assert end > start
    grid = np.arange(start, end + 1, 1_000_000_000 // args.fps, dtype=np.int64)
    count = len(grid)
    log(f"Common overlap: {count} frames, {count / args.fps:.2f} s, {args.fps} FPS")
    arrays = {}
    names = {}
    for key, group in (("observation.state", state_specs), ("action", action_specs)):
        parts = []
        names[key] = []
        for topic, (field_names, _) in group.items():
            times, values = telemetry[topic]["times"], telemetry[topic]["values"]
            if key == "action":
                indices = np.searchsorted(times, grid, side="right") - 1
                assert indices.min() >= 0
                aligned = values[indices]
            else:
                relative_times, relative_grid = (times - start) / 1e9, (grid - start) / 1e9
                aligned = np.column_stack([np.interp(relative_grid, relative_times, column) for column in values.T])
            parts.append(aligned)
            names[key].extend(field_names)
        arrays[key] = np.concatenate(parts, axis=1).astype(np.float32)
    assert arrays["observation.state"].shape == (count, 56)
    assert arrays["action"].shape == (count, 16)
    features = {key: {"dtype": "float32", "shape": (len(value),), "names": value} for key, value in names.items()}
    for alias, topic in camera_topics.items():
        detail = metadata["video_stream_details"][topic]
        features[f"observation.images.{alias}"] = {"dtype": "video", "shape": (detail["height"], detail["width"], 3), "names": ["height", "width", "channels"]}
    meta = LeRobotDatasetMetadata.create(repo_id=f"local/{args.output.name}", fps=args.fps,
        features=features, root=args.output, robot_type="dual_franka_fr3_robotiq_2f85")
    meta.add_task(task)
    arrays.update({"timestamp": (np.arange(count) / args.fps).astype(np.float32),
        "frame_index": np.arange(count, dtype=np.int64), "episode_index": np.zeros(count, dtype=np.int64),
        "index": np.arange(count, dtype=np.int64), "task_index": np.zeros(count, dtype=np.int64)})
    table = datasets.Dataset.from_dict(arrays, features=get_hf_features_from_features(meta.features))
    parquet_path = args.output / meta.get_data_file_path(0)
    parquet_path.parent.mkdir(parents=True, exist_ok=True)
    table.to_parquet(parquet_path)
    stats = compute_episode_stats(arrays, meta.features)
    alignment = {"target_timestamp_ns": grid}
    video_report = {}
    for alias, topic in camera_topics.items():
        key = f"observation.images.{alias}"
        log(f"Resampling and encoding {alias}")
        shape, stats[key], chosen, video_report[alias] = export_video(videos.pop(topic), camera_times[alias], grid,
            args.output / meta.get_video_file_path(0, key), args.fps)
        assert shape == features[key]["shape"]
        alignment[f"{alias}_source_frame_index"] = chosen
        alignment[f"{alias}_source_timestamp_ns"] = camera_times[alias][chosen]
    meta.update_video_info()
    meta.save_episode(0, count, [task], stats)
    modality = json.loads(args.modality.read_text())
    modality.pop("annotation", None)
    write_json(args.output / "meta/modality.json", modality)
    provenance = args.output / "provenance"
    provenance.mkdir()
    for name in ("episode_metadata.json", "record_metadata.json", "conversion_metadata.json"):
        shutil.copy2(args.input_episode / name, provenance / name)
    np.savez_compressed(provenance / "frame_alignment.npz", **alignment)
    report = {"source_episode_id": episode["episode_id"], "source_processed_mcap": str(source_file),
        "source_sha256": hashlib.file_digest(source_file.open("rb"), "sha256").hexdigest(),
        "task": task, "format": "LeRobot v2.1", "lerobot_version": "0.3.3", "fps": args.fps,
        "frames": count, "duration_seconds": count / args.fps, "synchronized_start_ns": start,
        "last_sample_ns": int(grid[-1]), "state_dimension": 56, "action_dimension": 16,
        "synchronization": {"clock": "MCAP log_time", "state": "linear interpolation",
            "action": "previous recorded command (zero-order hold)", "video": "nearest original frame timestamp",
            "extrapolation": False}, "video_codec": "h264", "video_crf": 18,
        "image_statistics": "all output frames, spatially subsampled pixels, RGB normalized to [0, 1]",
        "topics": topic_report, "videos": video_report}
    write_json(provenance / "conversion_report.json", report)
    log("Validating with the installed LeRobotDataset reader")
    dataset = LeRobotDataset(f"local/{args.output.name}", root=args.output, video_backend="pyav")
    assert len(dataset) == count and dataset.num_episodes == 1
    checked = sorted({0, 1, count // 4, count // 2, 3 * count // 4, count - 2, count - 1})
    for index in checked:
        row = dataset[index]
        np.testing.assert_allclose(row["observation.state"].numpy(), arrays["observation.state"][index])
        np.testing.assert_allclose(row["action"].numpy(), arrays["action"][index])
        assert row["task"] == task
        for alias in camera_topics:
            image = row[f"observation.images.{alias}"]
            assert np.isfinite(image.numpy()).all() and image.shape[0] == 3
    write_json(provenance / "validation.json", {"passed": True, "reader": "LeRobotDataset 0.3.3 / pyav",
        "frames": count, "episodes": 1, "sampled_row_indices": checked, "cameras_checked": list(camera_topics)})
    readme = f"""# {args.output.name}

Task: {task}

Source episode: `{episode['episode_id']}` (Tower of Babel, {episode['label']}).
LeRobot v2.1; 1 episode; {count} frames; {args.fps} FPS; {count / args.fps:.2f} seconds.
Three synchronized RGB MP4 videos at original resolution. H.264, CRF 18, GOP 2.

## Features

- `observation.state`: 56 floats; left arm then right arm, each containing 7 joint positions,
  7 joint velocities, 1 gripper knuckle position, 7 external joint torques, and 6 external wrench values.
- `action`: 16 floats; left arm's 7 target joint positions and gripper command, then right arm's 8 values.
- `observation.images.head`: 720 x 1280 RGB; wrist_left and wrist_right: 480 x 640 RGB.
- Joint positions/velocities remain in rad/rad/s; gripper state is the recorded knuckle joint position
  in rad, while gripper action is the recorded target width fraction (1=open). No unit normalization.
- `meta/modality.json` preserves the deployment's state/action slices and video keys.

## Timing and validation

The export uses the common time overlap, with a regular {args.fps} Hz grid on MCAP log timestamps.
States use linear interpolation, actions use the last recorded command, and images use the nearest
original frame timestamp. See `provenance/frame_alignment.npz` for every selected image index/time.
Videos are resampled using the original per-frame timestamps; there is no extrapolation.
Statistics are computed from this episode; no rewards or extra validity labels are synthesized.
Original metadata, conversion details, and loader validation are under `provenance/`.

## Load locally

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset
dataset = LeRobotDataset(
    "local/{args.output.name}",
    root="/home/ebim/labs/data/datasets/lerobot/{args.output.name}",
    video_backend="pyav",
)
sample = dataset[0]
```

Use LeRobot 0.3.3 (the labs dataset-builder image). The dataset is local only.
"""
    (args.output / "README.md").write_text(readme)
    (args.output / ".conversion_complete").touch()
    log(f"COMPLETE: {args.output}")


if __name__ == "__main__":
    main()
