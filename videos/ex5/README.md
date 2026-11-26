# Exercise 5 videos (manimgx)

Source for two explainer videos about the Exercise 5 ICP node (`ros2_ws/src/icp_odom`, fixed version):

| Scene | Length | What it covers |
|---|---|---|
| `Ex5Overview` in `ex5_overview.py` | 5:50 | the idea, 3D→2D slice, the three TASKs on real scans, fusion, results on the bag, the fixes |
| `Ex5Walkthrough` in `ex5_walkthrough.py` | 23:00 | `icp_node.cpp` block by block, every choice, ~25 Testat questions with answers, quick quiz |

Rendered at 1080p60 with an English narration and subtitles (`.srt` next to each video). The rendered videos and
the voice cache are not in git; render them with the steps below.

## Render

Needs [manimgx](https://github.com/academa-labs/manimgx) (Python 3.13+) and, for the voice, Windows with the
built-in voice "Microsoft David Desktop" (`winvoice.py` calls System.Speech through PowerShell).

```powershell
uv tool install --python 3.14 manimgx
cd videos/ex5
manimgx render ex5_overview.py Ex5Overview
manimgx render ex5_walkthrough.py Ex5Walkthrough
```

- `manimgx inspect <file> <Scene>` checks the layout and writes storyboards instead of a video.
- Each spoken line is synthesised once and kept in `voice/`; replace a WAV there with your own recording of the
  same words to use it instead. On Linux or macOS, set `voice` in the scene classes to another voice (manimgx's
  default is fal.ai, which needs `FAL_KEY`).
- Code excerpts are read from `../../ros2_ws/src/icp_odom/src/icp_node.cpp` by line number, so they match that
  file. Lines over 92 characters are shortened with "…" on screen.

## Data

All scans and numbers are real, from the course bag `dlg_feldtage_24:cut`:

- `data/scans.json`: the 800 `/ouster/scan` messages made by `pointcloud_to_laserscan` with the launch file's
  parameters, converted to `base_footprint` (the lidar frame `os_lidar` is turned 180° and 0.54 m up), plus `/odom`.
- `data/icp_demo.json`: ICP traces for a straight pair and a turning pair (0.5 s apart), with and without the
  initial guess, and the ICP-only path over the whole bag. Same algorithm as the C++ node; its ICP-only end pose
  matches the real node (x −0.06 m, y −22.28 m, −83.7°).

To rebuild them (Ubuntu with ROS 2 Jazzy, marina, and a workspace with `pointcloud_to_laserscan` built):

```bash
bash tools/record_scans.sh            # plays the bag once, writes data/scans.json (WS=~/ex5_ws by default)
python tools/prep_icp.py data/scans.json data/icp_demo.json   # numpy + scipy
```

## Files

| File | Role |
|---|---|
| `common.py` | colours, text helpers, code blocks with highlighted lines, scan drawing |
| `icp_anim.py` | views and dots for the ICP animations |
| `winvoice.py` | narration with the Windows voice |
| `ex5_overview.py`, `ex5_walkthrough.py` | the two scenes |
| `tools/` | recording and preprocessing of the bag data |
