"""Crop the captured RViz frames to the 3D view, for part 5.

    python prep_rviz.py IN_DIR OUT_DIR left top right bottom [last_frame]
The box is in pixels of the captured frames (the 3D view without RViz's panels).
"""
import sys
from pathlib import Path

from PIL import Image

src, dst = Path(sys.argv[1]), Path(sys.argv[2])
box = tuple(int(v) for v in sys.argv[3:7])
last = int(sys.argv[7]) if len(sys.argv) > 7 else None
dst.mkdir(parents=True, exist_ok=True)
for f in dst.glob("*.jpg"):
    f.unlink()
files = sorted(src.glob("*.jpg"))[: (last + 1) if last is not None else None]
for i, f in enumerate(files):
    Image.open(f).crop(box).save(dst / f"{i:05d}.jpg", quality=90)
print(len(files), "frames", Image.open(files[0]).crop(box).size)
