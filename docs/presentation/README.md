# ReloBot: An Engineering Journey

An interview-style account of how a broken mower became a ROS 2 robot, from field testing to Gazebo and the GSKB design workflow. Four media-only opening slides lead into seven English story slides with Russian speaker notes. Allow roughly 15–20 minutes for the story and live demo, plus the opener.

## Quick Start

### 1. View Directly (Zero-Install / Offline)
Keep `index.html`, `images/`, and `video/` together. The videos are local and Git-ignored, so a fresh checkout needs the MP4 files supplied separately. Double-click `index.html` or open via WSL:
```bash
wslview index.html
# Or open file:///path/to/docs/presentation/index.html in any browser
```

### 2. Live Editing & Watch Mode
When making changes inside `src/`:
```bash
# Automatically rebuild index.html whenever any slide, CSS, or JS file changes:
python3 build.py --watch

# Or run watch mode with a local dev server:
python3 build.py --watch --serve
```

### 3. Single Rebuild
```bash
python3 build.py
```

---

## Modular File Structure

```
docs/presentation/
├── index.html                      # Compiled presentation (relative media assets)
├── build.py                        # Zero-dependency Python compiler & live watcher
├── SPEAKER_NOTES_RU.md             # Russian interview cues, transitions, demo fallback
├── slides.md                       # Text-only Marp backup of the original six-slide story
├── images/                         # Gladiator, robot photo, Pi 5 case
├── video/                          # Local MP4s and source MKVs (Git-ignored)
└── src/                            # Modular source files (edit these!)
    ├── index.template.html         # HTML shell template (head, navbar, canvas, drawer, footer)
    ├── css/
    │   ├── variables.css           # Design tokens, color palette, fonts
    │   ├── base.css                # Resets, engineering grid background, slide stage
    │   ├── components.css          # Cards, drawer, timer, buttons, badges, pipelines
    │   ├── slides.css              # Custom layouts (Hub & Spoke, Live Demo, Incidents)
    │   └── main.css                # Root stylesheet importing all sub-modules
    ├── js/
    │   ├── notes.js                # On-screen Russian speaker notes (slides 5–11)
    │   └── presentation.js         # Keyboard navigation, stopwatch timer, drawer, fullscreen
    └── slides/
        ├── 00_windows_xp.html      # Slide 1: Windows XP video
        ├── 00a_gladiator.html      # Slide 2: Gladiator image
        ├── 00b_dreame.html        # Slide 3: Dreame video
        ├── 00c_worx.html          # Slide 4: Worx video
        ├── 01_overview.html        # Slide 5: Broken mower and surviving chassis
        ├── 02_hardware.html        # Slide 6: ROS 2 contracts and hub & spoke
        ├── 03_software.html        # Slide 7: Synergy, field tests, Gazebo
        ├── 04_intelligence.html    # Slide 8: GSKB vs. code factories
        ├── 05_demo.html            # Slide 9: Gazebo, UI, Petrovich/GSKB live demo
        ├── 05a_system_flow.html    # Slide 10: GSKB development harness and robot control flow
        └── 06_engineering.html     # Slide 11: Next challenges and questions
```

---

    ## Opening Videos

    Slides 1, 3, and 4 play muted and loop while visible; click or tap to pause/resume. The four opening slides contain only media, fill the window, and may crop edges on non-16:9 screens. The presenter bar and notes stay hidden until slide 5; use the keyboard to advance.

    Prepare browser-compatible MP4s from the local MKVs if the MP4 files are missing (requires FFmpeg):

    ```bash
    cd docs/presentation
    for clip in MicrosoftWindowsXPBlissWallpaperAnimated DreameX60Ultrareview WorxLandroidInstallationGuide; do
      ffmpeg -i "video/$clip.mkv" -map 0:v:0 -c:v copy -an -movflags +faststart "video/$clip.mp4"
    done
    ```

    This copies the H.264 video without re-encoding; sound is omitted because playback is intentionally muted. Share the presentation with the `images/` and `video/` folders at those paths, and test playback offline on the first speaker's browser.

    ---

## Keyboard Controls
| Key | Action |
| :--- | :--- |
| `→` / `Space` / `PageDown` | Next slide |
| `←` / `PageUp` | Previous slide |
| `S` | Toggle Russian Speaker Notes Drawer |
| `F` | Toggle Fullscreen Mode |
| Click on Stopwatch | Reset Presentation Timer to `00:00` |

## Live Demo

From the repository root, start the simulation with `./start_sim.sh up --gui`. Once the stack is ready, slide 9 opens the robot dashboard at `http://localhost/`. The chat is named **Petrovich**; its **GSKB** button opens the AGY terminal. Slide 10 contrasts the GSKB development loop, bounded by skills, instructions, memory and existing code, with the physical robot's control path; the demo substitutes Gazebo for the Pico wheel controller and motors. The presentation itself works offline with its local media, but the dashboard and agent require their respective services. Rehearse the workflow and prepare a saved map and code diff for the fallback described in `SPEAKER_NOTES_RU.md`.
