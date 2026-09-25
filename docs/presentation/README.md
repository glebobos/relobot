# ReloBot: An Engineering Journey

An interview-style account of how a broken mower became a ROS 2 robot, from field testing to Gazebo and the GSKB design workflow. The six English slides are visual cues; one Russian speaker's notes carry the story. Allow roughly 15–20 minutes including the live demo.

## Quick Start

### 1. View Directly (Zero-Install / Offline)
Double-click `index.html` or open via WSL:
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
├── index.html                      # Standalone, compiled single-file presentation (portable)
├── build.py                        # Zero-dependency Python compiler & live watcher
├── SPEAKER_NOTES_RU.md             # Russian interview cues, transitions, demo fallback
├── slides.md                       # Short Marp backup of the six-slide story
├── images/                         # Media assets (robot photo, Pi 5 case)
└── src/                            # Modular source files (edit these!)
    ├── index.template.html         # HTML shell template (head, navbar, canvas, drawer, footer)
    ├── css/
    │   ├── variables.css           # Design tokens, color palette, fonts
    │   ├── base.css                # Resets, engineering grid background, slide stage
    │   ├── components.css          # Cards, drawer, timer, buttons, badges, pipelines
    │   ├── slides.css              # Custom layouts (Hub & Spoke, Live Demo, Incidents)
    │   └── main.css                # Root stylesheet importing all sub-modules
    ├── js/
    │   ├── notes.js                # On-screen Russian speaker notes (slides 1–6)
    │   └── presentation.js         # Keyboard navigation, stopwatch timer, drawer, fullscreen
    └── slides/
        ├── 01_overview.html        # Slide 1: Broken mower and surviving chassis
        ├── 02_hardware.html        # Slide 2: ROS 2 contracts and hub & spoke
        ├── 03_software.html        # Slide 3: Synergy, field tests, Gazebo
        ├── 04_intelligence.html    # Slide 4: GSKB vs. code factories
        ├── 05_demo.html            # Slide 5: Gazebo, UI, Petrovich/GSKB live demo
        └── 06_engineering.html     # Slide 6: Next challenges and questions
```

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

From the repository root, start the simulation with `./start_sim.sh up --gui`. Once the stack is ready, slide 5 opens the robot dashboard at `http://localhost/`. The chat is named **Petrovich**; its **GSKB** button opens the AGY terminal. The presentation itself works offline, but the dashboard and agent require their respective services. Rehearse the workflow and prepare a saved map and code diff for the fallback described in `SPEAKER_NOTES_RU.md`.
