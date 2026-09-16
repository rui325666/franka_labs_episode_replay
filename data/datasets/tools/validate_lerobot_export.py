#!/usr/bin/env python3
"""Independent full-file integrity checks for the generated single-episode dataset."""
import hashlib
import json
from pathlib import Path
import sys

import av
import numpy as np
import pyarrow.parquet as pq


root = Path(sys.argv[1])
info = json.loads((root / "meta/info.json").read_text())
episode = json.loads((root / "meta/episodes.jsonl").read_text())
stats = json.loads((root / "meta/episodes_stats.jsonl").read_text())["stats"]
modality = json.loads((root / "meta/modality.json").read_text())
count = info["total_frames"]
fps = info["fps"]
assert info["codebase_version"] == "v2.1"
assert info["total_episodes"] == 1 and episode["length"] == count
table = pq.read_table(root / "data/chunk-000/episode_000000.parquet")
assert table.num_rows == count
nonvideo = {k for k, f in info["features"].items() if f["dtype"] != "video"}
assert set(table.column_names) == nonvideo
for key in nonvideo:
    values = np.array(table[key].to_pylist())
    assert np.isfinite(values).all(), key
    assert values.shape == ((count,) if info["features"][key]["shape"] == [1]
                            else (count, *info["features"][key]["shape"])), key
    for name, expected in (("min", values.min(axis=0)), ("max", values.max(axis=0)),
                           ("mean", values.mean(axis=0)), ("std", values.std(axis=0))):
        # Numeric statistics are originally computed on float32 episode arrays.
        np.testing.assert_allclose(np.asarray(stats[key][name]), np.atleast_1d(expected), rtol=5e-5, atol=2e-5)
    assert stats[key]["count"] == [count], key
np.testing.assert_allclose(table["timestamp"].to_numpy(), np.arange(count) / fps, atol=1e-5, rtol=0)
np.testing.assert_array_equal(table["frame_index"].to_numpy(), np.arange(count))
np.testing.assert_array_equal(table["index"].to_numpy(), np.arange(count))
assert np.all(table["episode_index"].to_numpy() == 0)
assert np.all(table["task_index"].to_numpy() == 0)
for section, key in (("state", "observation.state"), ("action", "action")):
    slices = list(modality[section].values())
    assert slices[0]["start"] == 0
    assert all(a["end"] == b["start"] for a, b in zip(slices, slices[1:]))
    assert slices[-1]["end"] == info["features"][key]["shape"][0]
videos = {}
for key, feature in info["features"].items():
    if feature["dtype"] != "video":
        continue
    path = root / info["video_path"].format(episode_chunk=0, video_key=key, episode_index=0)
    with av.open(str(path)) as video:
        stream = video.streams.video[0]
        assert abs(float(stream.average_rate) - fps) < 1e-6
        assert [stream.height, stream.width, 3] == feature["shape"]
        decoded = 0
        for i, frame in enumerate(video.decode(video=0)):
            assert abs(float(frame.pts * frame.time_base) - i / fps) < 1e-6
            assert [frame.height, frame.width, 3] == feature["shape"]
            decoded += 1
        assert decoded == count, (key, decoded, count)
    assert stats[key]["count"] == [count]
    for name in ("min", "max", "mean", "std"):
        array = np.array(stats[key][name])
        assert array.shape == (3, 1, 1) and np.isfinite(array).all()
    videos[key] = {"decoded_frames": decoded, "fps": fps, "shape": feature["shape"]}
    print(f"PASS {key}: all {decoded} frames decoded, timestamps and dimensions match", flush=True)
alignment = np.load(root / "provenance/frame_alignment.npz")
assert len(alignment["target_timestamp_ns"]) == count
assert np.all(np.diff(alignment["target_timestamp_ns"]) == 1_000_000_000 // fps)
hashes = {}
for path in sorted(root.rglob("*")):
    if path.is_file() and path.parts[-2] != "provenance" and path.name != ".conversion_complete":
        with path.open("rb") as handle:
            hashes[str(path.relative_to(root))] = hashlib.file_digest(handle, "sha256").hexdigest()
report = {"passed": True, "parquet_rows": count, "numeric_features": sorted(nonvideo),
    "video_full_decode": videos, "checked": ["finite values", "feature shapes", "numeric statistics",
    "contiguous indices", "20 Hz timestamps", "modality slices", "every video frame"], "sha256": hashes}
(root / "provenance/integrity_validation.json").write_text(json.dumps(report, indent=2) + "\n")
print("PASS: complete dataset integrity validation", flush=True)
