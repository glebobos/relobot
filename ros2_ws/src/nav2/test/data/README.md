# Recorded Ingress Regression

`ingress_replay.npz` contains only the first nonempty execution path and map
from the local 20260930T075941Z continuous-v1 recording. It contains no command
publishers or executable content. Arrays: poses (x, y, yaw), occupancy,
resolution, and map origin (x, y, yaw).

The synthetic controller test begins at path index 28, with the observed
0.024 m cross-track and -0.012 rad heading error. It must pass the approach
turn, stay within the corridor, and respect both raw and smoothed velocity
bounds. This reproduces the old progress abort on the captured map, not on
an all-free substitute map. It is an isolated regression, not a Gazebo trial.