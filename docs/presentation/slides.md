---
marp: true
theme: uncover
class: invert
paginate: true
header: 'ReloBot · An engineering journey'
footer: 'ROS 2 · Docker Compose · Gazebo'
style: |
  section {
    background: #0b0f14;
    color: #f1f5f9;
    font-family: 'IBM Plex Sans', sans-serif;
    text-align: left;
    padding: 46px 60px;
  }
  h1 { color: #f1f5f9; font-size: 2em; }
  h2 { color: #2ebd85; font-size: 1.25em; }
  p, li { color: #cbd5e1; }
  code { color: #2ebd85; }
  img { max-height: 310px; max-width: 100%; }
---

<!-- Slide 1 -->
# How it started

![The chassis that became ReloBot](images/robot_start.jpg)

**Dead board. Surviving mechanics. A decision to rebuild.**

---

<!-- Slide 2 -->
# Choosing ROS 2 and boundaries

## Hub & spoke · plug & play · explicit contracts

Raspberry Pi 5 **hub** ↔ USB / micro-ROS **spokes**

ROS 2 contracts · Docker Compose profiles

---

<!-- Slide 3 -->
# From the field to Gazebo

World · Chassis · Controllers · Code

**Small QoS change → unexpected system behavior**

Nearly two years of field testing → repeatable tests in Gazebo.

Simulated physics is an earlier test, not a copy of the real lawn.

---

<!-- Slide 4 -->
# A design bureau, not a factory

**Intent → Contracts → GSKB → Code → Evidence**

The production line repeats. The design bureau explores and checks.

---

<!-- Slide 5 -->
# Watch it change live

1. Start Gazebo in VS Code.
2. Open the robot UI: `http://localhost/`.
3. Meet **Petrovich** and open **GSKB**.
4. Ask for a map-camera toggle; inspect the diff.

Fallback: saved map and reviewed change.

---

<!-- Slide 6 -->
# A platform still learning

- See obstacles below the LiDAR.
- Learn to recover safely.
- Do more with the same computer.
- Widen hardware support through contracts.

**Questions?**

<!-- Russian interview cues and demo checklist: SPEAKER_NOTES_RU.md -->