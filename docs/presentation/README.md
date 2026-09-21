# ReloBot Presentation & Architecture Review

Interactive, modern technical architecture presentation for the **ReloBot Platform / DarkSelmash AI Factory**.

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
├── SPEAKER_NOTES_RU.md             # Complete Russian speaker cue sheet & speech reference
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
    │   ├── notes.js                # Speaker notes dictionary mapped by slide index (1–7)
    │   └── presentation.js         # Keyboard navigation, stopwatch timer, drawer, fullscreen
    └── slides/
        ├── 01_overview.html        # Slide 1: Platform Overview & Specs
        ├── 02_hardware.html        # Slide 2: Hub & Spoke Hardware Network
        ├── 03_software.html        # Slide 3: Containerized ROS 2 & Gazebo Twin
        ├── 04_intelligence.html    # Slide 4: RoboFactory AGY Agent & Piper TTS
        ├── 05_demo.html            # Slide 5: Live Dashboard & Telemetry Demo
        ├── 06_engineering.html     # Slide 6: Hard Problems Solved in Field
        └── 07_roadmap.html         # Slide 7: Roadmap & Technical Discussion
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
