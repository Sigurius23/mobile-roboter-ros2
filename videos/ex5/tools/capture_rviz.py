"""Grab the RViz window (shown by WSLg on the Windows desktop) as JPEG frames, for part 5.

Usage (Windows, manimgx's Python, which has Pillow):
    python capture_rviz.py OUT_DIR [seconds] [fps]
Waits for a window whose title contains "RViz", then grabs its client area for `seconds`.
The window must stay visible (not covered) while it records.
"""
import ctypes
import ctypes.wintypes as wt
import sys
import time
from pathlib import Path

from PIL import ImageGrab

user32 = ctypes.windll.user32
user32.SetProcessDPIAware()


def find(title_part):
    found = []

    @ctypes.WINFUNCTYPE(ctypes.c_bool, wt.HWND, wt.LPARAM)
    def cb(hwnd, _):
        if user32.IsWindowVisible(hwnd):
            n = user32.GetWindowTextLengthW(hwnd)
            buf = ctypes.create_unicode_buffer(n + 1)
            user32.GetWindowTextW(hwnd, buf, n + 1)
            if title_part in buf.value:
                found.append(hwnd)
        return True

    user32.EnumWindows(cb, 0)
    return found[0] if found else None


def client_box(hwnd):
    r = wt.RECT()
    user32.GetClientRect(hwnd, ctypes.byref(r))
    pt = wt.POINT(0, 0)
    user32.ClientToScreen(hwnd, ctypes.byref(pt))
    return pt.x, pt.y, pt.x + r.right, pt.y + r.bottom


def main():
    out = Path(sys.argv[1])
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 60
    fps = float(sys.argv[3]) if len(sys.argv) > 3 else 10
    out.mkdir(parents=True, exist_ok=True)
    for f in out.glob("*.jpg"):
        f.unlink()
    t0 = time.time()
    hwnd = None
    while hwnd is None and time.time() - t0 < 90:
        hwnd = find("RViz")
        time.sleep(0.5)
    if hwnd is None:
        sys.exit("no RViz window")
    user32.SetForegroundWindow(hwnd)
    time.sleep(1.0)
    start = time.time()
    print("START", start, client_box(hwnd), flush=True)
    i = 0
    while time.time() - start < secs:
        due = start + i / fps
        if time.time() < due:
            time.sleep(due - time.time())
        if not user32.IsWindow(hwnd):
            break
        img = ImageGrab.grab(bbox=client_box(hwnd), all_screens=True)
        img.convert("RGB").save(out / f"{i:05d}.jpg", quality=88)
        i += 1
    print("FRAMES", i, "in", round(time.time() - start, 1), "s", flush=True)


if __name__ == "__main__":
    main()
