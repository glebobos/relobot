---
trigger: always_on
description: Robot Control Action Recipes for One-Shot Execution (dock, undock, explore, stop)
---

# Robot Control Rules (One-Shot Execution)

When commanded to dock, undock, explore, or stop, execute IMMEDIATELY in one shot.
DO NOT call `get_actions`, `get_action_details`, `get_parameters`, `get_topics`, or `view_file` to search or inspect definitions or schema files. All required schemas, action names, action types, and argument structures are fully specified below.

## Action & Topic Recipes:

1. **DOCK** ("dock", "dock back", "go home", "return to dock", "recharge"):
   - Tool: `call_mcp_tool`
   - ServerName: `ros-mcp`
   - ToolName: `send_action_goal`
   - Arguments:
     ```json
     {
       "action_name": "/dock_robot",
       "action_type": "opennav_docking_msgs/action/DockRobot",
       "goal": {
         "use_dock_id": true,
         "dock_id": "home_dock",
         "navigate_to_staging_pose": true,
         "max_staging_time": 60.0
       }
     }
     ```

2. **UNDOCK** ("undock", "undock body", "leave dock", "step off", "get out"):
   - Tool: `call_mcp_tool`
   - ServerName: `ros-mcp`
   - ToolName: `send_action_goal`
   - Arguments:
     ```json
     {
       "action_name": "/navigate_to_pose",
       "action_type": "nav2_msgs/action/NavigateToPose",
       "goal": {
         "behavior_tree": "/ros2_ws/install/nav2/share/nav2/behavior_trees/undock_and_turn.xml",
         "pose": {
           "header": { "frame_id": "map" },
           "pose": {
             "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
             "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
           }
         }
       }
     }
     ```

3. **EXPLORE** ("explore", "start exploring", "explore area", "resume exploration"):
   - Tool: `call_mcp_tool`
   - ServerName: `ros-mcp`
   - ToolName: `publish_once`
   - Arguments:
     ```json
     {
       "topic": "/explore/resume",
       "msg_type": "std_msgs/msg/Bool",
       "msg": { "data": true }
     }
     ```

4. **STOP EXPLORING** ("pause exploring", "stop exploration", "halt exploration"):
   - Tool: `call_mcp_tool`
   - ServerName: `ros-mcp`
   - ToolName: `publish_once`
   - Arguments:
     ```json
     {
       "topic": "/explore/resume",
       "msg_type": "std_msgs/msg/Bool",
       "msg": { "data": false }
     }
     ```

5. **CANCEL DOCKING** ("cancel dock", "abort docking"):
   - Tool: `call_mcp_tool`
   - ServerName: `ros-mcp`
   - ToolName: `cancel_action_goal`
   - Arguments:
     ```json
     {
       "action_name": "/dock_robot"
     }
     ```

6. **EMERGENCY STOP** ("stop", "halt", "freeze", "kill motors"):
   - Tool: `call_mcp_tool`
   - ServerName: `ros-mcp`
   - ToolName: `publish_once`
   - Arguments:
     ```json
     {
       "topic": "/cmd_vel",
       "msg_type": "geometry_msgs/msg/Twist",
       "msg": {
         "linear": { "x": 0.0, "y": 0.0, "z": 0.0 },
         "angular": { "x": 0.0, "y": 0.0, "z": 0.0 }
       }
     }
     ```
