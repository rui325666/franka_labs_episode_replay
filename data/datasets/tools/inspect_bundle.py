"""Inspect and verify the bundle checksum using only the Python standard library."""
import argparse
import hashlib
import json
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("bundle", type=Path)
args = parser.parse_args()
manifest = json.loads((args.bundle / "manifest.json").read_text())
digest = hashlib.sha256((args.bundle / "episode.npz").read_bytes()).hexdigest()
if digest != manifest["bundle_sha256"]:
    raise SystemExit("Replay bundle checksum mismatch; do not use this bundle.")
print(f'Episode {manifest["episode"]}: {manifest["frames"]} frames, {manifest["fps"]:g} Hz, both arms')
print(json.dumps(manifest, indent=2, ensure_ascii=False))
