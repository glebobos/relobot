export const CFG = {
    turnAxis: 0,
    driveAxis: 3,
    knifeButton: 7,
    cruiseButton: 4,
    scaleLinear: 0.3,
    scaleAngular: 1.0,
    invertTurn: true,
    invertDrive: true,
    invertKnife: false,
    knifeMinRpm: 500,
    knifeMaxRpm: 3000,
};

export const BT_DIR = '/ros2_ws/install/nav2/share/nav2/behavior_trees';
export const DOCK_ID = 'home_dock';

export const TOPICS = {
    // Telemetry
    JOINT_STATES: '/joint_states',
    BATTERY: '/battery_voltage',
    KNIVES: '/knives/current_rpm',
    CHARGER: '/charger_voltage',
    ON_DOCK: '/on_dock',
    IMU: '/imu',
    ROSOUT: '/rosout',
    ODOMETRY: '/odometry/filtered',

    // Camera
    CAMERA_IMAGE: '/camera/image_rect',

    // Control / Drive
    CMD_VEL: '/cmd_vel',
    CMD_KNIVES: '/knives/set_rpm',

    // Exploration
    EXPLORE_RESUME: '/explore/resume',
    EXPLORE_STATUS: '/explore/status',

    // Coverage
    COVERAGE_COMMAND: '/coverage/command',
    COVERAGE_STATUS: '/coverage/status',
    COVERAGE_PREVIEW_PATH: '/coverage/preview_path',
    COVERAGE_POLYGON_ACTIVE: '/coverage/polygon_active',
    COVERAGE_OBSTACLES_ACTIVE: '/coverage/obstacles_active',

    // System
    ROBOT_DESCRIPTION: '/robot_description',
    DOCK_ACTION_STATUS: '/dock_robot/_action/status',
    SYSTEM_METRICS: '/system/metrics',
    SYSTEM_COMMAND: '/system/command'
};

/**
 * Commands published to TOPICS.COVERAGE_COMMAND.
 * The backend coverage node dispatches on these string values.
 */
export const CMDS = {
    COVERAGE_PREVIEW: 'preview',
    COVERAGE_EXECUTE: 'execute',
    COVERAGE_CANCEL: 'cancel',
    COVERAGE_REFRESH_MAP: 'refresh_map',
    RESET_MAP: 'reset_map',
    RESTART_SLAM: 'restart_slam',
    REBOOT: 'reboot',
    POWEROFF: 'poweroff',
};

/**
 * ROS message / action / service type strings (rosbridge shorthand).
 * Single source of truth — import here instead of hardcoding at call sites.
 */
export const MSG_TYPES = {
    // std_msgs
    FLOAT32: 'std_msgs/Float32',
    BOOL: 'std_msgs/Bool',
    STRING: 'std_msgs/String',
    // geometry_msgs
    TWIST: 'geometry_msgs/Twist',
    POLYGON_STAMPED: 'geometry_msgs/PolygonStamped',
    // nav_msgs
    PATH: 'nav_msgs/Path',
    ODOMETRY: 'nav_msgs/Odometry',
    // sensor_msgs
    IMU: 'sensor_msgs/Imu',
    // rcl_interfaces
    LOG: 'rcl_interfaces/Log',
    // action_msgs
    GOAL_STATUS_ARRAY: 'action_msgs/GoalStatusArray',
    // Custom / third-party
    EXPLORE_STATUS: 'explore_lite_msgs/ExploreStatus',
    DOCK_ROBOT: 'opennav_docking_msgs/action/DockRobot',
    NAVIGATE_TO_POSE: 'nav2_msgs/action/NavigateToPose',
    SERIALIZE_MAP_SRV: 'slam_toolbox/srv/SerializePoseGraph',
    SET_PARAMETERS: 'rcl_interfaces/srv/SetParameters',
};

/**
 * Status strings received on TOPICS.EXPLORE_STATUS.
 */
export const EXPLORE_STATUS = {
    STARTED: 'exploration_started',
    IN_PROGRESS: 'exploration_in_progress',
};

export const ACTIONS = {
    DOCK_ROBOT: '/dock_robot',
    NAVIGATE_TO_POSE: '/navigate_to_pose'
};

export const SERVICES = {
    SERIALIZE_MAP: '/slam_toolbox/serialize_map',
    SET_APRILTAG_PARAMETERS: '/apriltag_manager/set_parameters'
};
