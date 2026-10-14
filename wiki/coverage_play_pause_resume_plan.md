# Coverage Play, Pause And Resume Plan

Date: 2026-10-06
Status: Proposed; the UI and status-query changes below are not implemented.

## Goal

Reuse the current coverage Execute button as a play/pause control. Starting a
new route and resuming its retained remainder must be distinct commands. Keep
Emergency Stop separate and available during all operations.

| Backend condition | Icon | Command | Tooltip |
| --- | --- | --- | --- |
| Ready preview, no interrupted remainder | Play | `execute` | Execute Coverage |
| Confirmed interruption with retained remainder | Play | `resume` | Resume Coverage |
| Following a route or performing backup recovery | Pause | `cancel` | Pause Coverage |
| Planning, validating ingress/recovery, cancel requested or awaiting confirmation | Spinner, disabled | None | Waiting |
| No route or no confirmed current status | Play, disabled | None | Coverage Unavailable |

When both execute and resume are available, resume takes priority. Never fall
back from rejected resume to executing the full preview automatically.

## Existing Behavior To Reuse

- Backend accepts `execute`, `resume` and `cancel` on `/coverage/command`.
- Cancel retains the route and ordered cursor. Completion of the action, not
  the cancel response alone, confirms that execution has stopped.
- Resume revalidates the retained remainder and plans a fresh forward ingress.
- Operator cancellation must not trigger automatic backup recovery.
- New preview, zone changes, clear and map reset invalidate retained context.
- No separate paused state is required: confirmed cancellation plus available
  retained remainder is sufficient to offer Resume.

## Frontend Changes

1. Replace the initial seedling icon with the existing Font Awesome play icon.
   Switch between play, pause and spinner without changing button dimensions.
   Update tooltip and accessible label with the selected action.
2. Select the command from the most recent backend snapshot, never from the
   button's CSS class. Keep pending-command state separate from backend state.
3. Immediately lock duplicate clicks while a command is pending. A late
   progress message must not unlock a pending pause. Show Pause only when
   execution is confirmed; offer Resume only after confirmed cancellation or
   abort with retained route.
4. Pause sends coverage `cancel` only; do not reuse the broad Emergency Stop
   handler. Keep Emergency Stop's existing motor, knife, navigation, docking
   and exploration cancellation behavior unchanged.
5. Clear pending UI context on invalidation or loss of connection. Do not use
   localStorage as authoritative mission state, send motion on page load, or
   restart a mission automatically.

Frontend files: `ros2_ws/src/web_server/frontend/index.html`,
`ros2_ws/src/web_server/frontend/js/shared/constants.js` and
`ros2_ws/src/web_server/frontend/js/ui/control-panel/control-panel.js`.
Use `ros2_ws/src/web_server/frontend/js/services/ros-service.js` connection
events if needed for reconnect handling; preserve shared subscription behavior.

## Reload And Reconnect

UI-only command switching is possible, but reliable recovery of the current
button state after reload requires a backend snapshot. `/coverage/status` is
currently VOLATILE; the manager timer does not republish it. Progress while
following cannot substitute for a snapshot when the robot is paused or idle.

Add one read-only `get_status` command to the existing manager command channel:

- Permit it before the busy-command rejection gate, including during planning,
  execution, validation and cancellation.
- Return current operational `state`, `phase`, `can_execute`, `can_resume` and
  `can_pause` on `/coverage/status`. Include these fields on normal updates too.
- Compute availability from actual retained route/cursor and execution state,
  not merely the last command error or a browser-side remembered state. Avoid
  copying or fully validating a dense route just to answer the query.
- `can_resume` requires a confirmed blocked/canceled execution and a usable
  retained remainder. Freshness, collision and ingress checks still run when
  resume is explicitly requested; the flag is not permission to bypass them.
- Do not mutate route, cursor, action handles or planning/recovery state while
  answering the query. Do not start navigation, exploration or knives.

On load or reconnect, show a disabled waiting control, subscribe to status,
then request the snapshot. Use bounded retries for a lost initial response;
keep the control disabled if synchronization fails. Emergency Stop remains
available whenever the connection can carry its commands. A snapshot received
during a pending command is not by itself proof that the command completed.

Backend files for this small protocol addition:
`ros2_ws/src/nav2/frontier_explorer/coverage_manager.py` and, only if a retained
route availability helper is needed,
`ros2_ws/src/nav2/frontier_explorer/coverage_execution.py`.

## Verification

- Backend tests: snapshot queries during idle, preview, active action, pending
  cancellation and blocked/canceled remainder; query must not mutate state or
  send an action. Test empty/completed/invalidated remainder and clock reset.
- Frontend tests with mocked topics/statuses: execute, pause, resume, duplicate
  clicks, delayed confirmation, late progress, command refusal, new preview,
  zone changes, load/reconnect while paused and unavailable backend.
- Reuse existing coverage cancellation tests for pending goal acceptance,
  validation workers and recovery; preserve stop confirmation and progress.
- Browser check: icons, tooltips, accessible labels and stable layout on desktop
  and mobile. Intercept commands or use mocks; do not command the live robot.
- Run focused ROS tests through `helpers/navigation_diagnostics/lab.sh` inside
  Docker. Do not build or run ROS nodes directly on the host.

## Rollout And Scope

Frontend files are volume-served by Nginx; refresh the page without restarting
the stack. The proposed backend snapshot addition requires a separate planned
Nav2 reload; frontend refresh alone cannot load new Python backend behavior.
Coordinate that reload before an active mission because retained coverage
state is currently in memory.

Disk checkpoints, resuming across Nav2/stack restart, new controller tuning,
mission scheduling, firmware changes and UI redesign are outside this plan.
Page reload does not restart Nav2; stack restart loses retained route/cursor.