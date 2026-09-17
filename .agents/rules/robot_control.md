# Robot Control Rules (One-Shot Execution)

When commanded to dock, undock, explore, or stop, execute IMMEDIATELY in one shot.
DO NOT call `get_actions`, `get_action_details`, `get_parameters`, `get_topics`, or `view_file` to search or inspect definitions first.

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
       "action_name": "/undock_robot",
       "action_type": "opennav_docking_msgs/action/UndockRobot",
       "goal": {
         "dock_type": "apriltag_dock",
         "max_undocking_time": 30.0
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
