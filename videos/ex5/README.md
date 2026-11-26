# Exercise 5 videos, slides and code explorer (manimgx)

Study material for the Exercise 5 ICP node (`ros2_ws/src/icp_odom`, fixed version).

## What's here

| What | Where | Notes |
|---|---|---|
| Overview video, 5:50 | `Ex5Overview` in `ex5_overview.py` | the idea, the three TASKs on real scans, results, the fixes |
| Line-by-line walkthrough, 5 parts, ~48 min | `Part1Node` … `Part5Extras` in `ex5_walk.py` | every line with what, why and what goes wrong otherwise; Testat questions; Part 5 has a real RViz recording |
| Old walkthrough, 23:00 | `Ex5Walkthrough` in `ex5_walkthrough.py` | the first, coarser version (Windows voice), kept for reference |
| Slide deck, 37 slides with speaker notes | `deck/ICP_Node_Walkthrough.html` (offline copy) | live deck with Present mode and PowerPoint/PDF export: <https://claude.ai/artifact/L8ahk2hNVJVEC5VsPUUStM> (private to the owner) |
| Code explorer with hover explanations | `explorer.html` | open in any browser; hover (or tap) underlined code. Online copy: <https://claude.ai/artifact/7kw7yMKe887ie4zzRppLsk> |

The rendered videos (`*.mp4`, about 200 MB) and the voice cache are **not in git**: copy them by hand or render
them again with the steps below. `deck/ICP_Node_Walkthrough.html` and `explorer.html` open offline; with internet
they use the web fonts, without it the fallback fonts.

## Render the videos

Needs [manimgx](https://github.com/academa-labs/manimgx) (Python 3.13+).

```powershell
uv tool install --python 3.14 manimgx
cd videos/ex5
manimgx render ex5_walk.py Part1Node        # Part2Frames, Part3ICP, Part4Fusion, Part5Extras
manimgx render ex5_overview.py Ex5Overview
```

- **Voice.** The five walkthrough parts use [Phonon TTS](https://github.com/gradium-ai/xn-ptts) (`phononvoice.py`,
  default English model Phonon 31m, Apache 2.0, runs on the CPU). Install it into manimgx's own environment:
  `uv pip install ptts --python <uv tool dir>/manimgx/Scripts/python.exe`. The model (~65 MB) downloads from
  Hugging Face on the first render; set `HF_HOME` to choose where. The overview and the old walkthrough use the
  Windows voice (`winvoice.py`, Windows only).
- Each spoken line is synthesised once and kept in `voice/`.
- `manimgx inspect <file> <Scene> -t 30,60` checks the layout at given times instead of rendering.
- Code excerpts are read from `../../ros2_ws/src/icp_odom/src/icp_node.cpp` by line number, so they always match it.
- In the narration, `[brackets]` mark the words each animation starts on (`fine.Walk.step` checks the count).

## Data

All scans and numbers are real, from the course bag `dlg_feldtage_24:cut`:

- `data/scans.json`: the 800 `/ouster/scan` messages made by `pointcloud_to_laserscan` with the launch file's
  parameters, converted to `base_footprint` (the lidar frame `os_lidar` is turned 180° and 0.54 m up), plus `/odom`.
- `data/icp_demo.json`: ICP traces for a straight pair and a turning pair (0.5 s apart), with and without the
  initial guess, and the ICP-only path over the whole bag.
- `data/rviz_clip/`: the RViz screen recording used in Part 5 (255 frames, cropped to the 3D view).

To rebuild them (Ubuntu with ROS 2 Jazzy, marina, and a workspace with `pointcloud_to_laserscan` and `icp_odom` built):

```bash
bash tools/record_scans.sh            # plays the bag once, writes data/scans.json (WS=~/ex5_ws by default)
python tools/prep_icp.py data/scans.json data/icp_demo.json   # numpy + scipy
```

RViz recording for Part 5 (Windows + WSL, run both at the same time):

```powershell
wsl -d Ubuntu-24.04 -- bash -l /mnt/e/aufgaben-public/videos/ex5/tools/rviz_run.sh   # node + bag + RViz
python tools/capture_rviz.py data/rviz_frames 62 10      # grabs the RViz window (Pillow); keep it uncovered
python tools/prep_rviz.py data/rviz_frames data/rviz_clip 404 120 1321 791 254   # crop to the 3D view
```

## Files

| File | Role |
|---|---|
| `ex5_walk.py` | the five walkthrough parts |
| `parts.py` | shared scene helpers: title cards, the "Where are we?" map, plain cards, Q&A |
| `fine.py` | `Walk`: one code window, line highlight, token labels, C++/WHY boxes, word-timed |
| `context.py` | the map of the node used in "Where are we?" |
| `common.py` | colours, text, code blocks, scan drawing |
| `icp_anim.py` | views and dots for the ICP animations |
| `phononvoice.py`, `winvoice.py` | narration voices |
| `ex5_overview.py`, `ex5_walkthrough.py` | the overview and the old walkthrough |
| `deck/project/` | slide sources (`deck.json`, `deck.css`, one HTML file per slide, notes in `<aside>`) |
| `tools/build_deck_html.py` | rebuilds `deck/ICP_Node_Walkthrough.html` from `deck/project/` |
| `explorer.html` | the hover code explorer |
| `tools/` | recording and preprocessing of the bag data, RViz recording |
