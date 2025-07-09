Here you can search for a topic or service and find the corresponding consumer

Consumers handle communication to ROS2 topics and services, as well as frontend websockets


## ArmConsumer
`node_name = teleop_arm`

websocket: `arm`

### Publishing
- `arm_throttle_cmd`
- `ee_pos_cmd`
- `ee_vel_cmd`
- `joystick_cmd_vel`
- `controller_cmd_vel`
- `mast_gimbal_throttle_cmd`
- `sa_throttle_cmd`

### Forwarding
- `arm_controller_state`
- `arm_joint_data`

### Receiving
- `joystick`
- `mast_keyboard`
- `ra_controller`
- `ra_mode`
- `sa_controller`
- `sa_mode`

---


## AutonConsumer
`node_name = teleop_auton`

websocket: `auton`

### Services
- `enable_teleop`
- `enable_auton`

### Receiving
- `teleop_enable`
- `auton_enable`

---


## DriveConsumer
`node_name = teleop_drive`

websocket: `drive`

### Forwarding
- `drive_left_controller_data`
- `drive_right_controller_data`
- `drive_controller_data`

---


## NavConsumer
`node_name = teleop_nav`

websocket: `nav`

### Forwarding
- `nav_state`
- `gps/fix`
- `basestation/position`
- `drone_odometry`

---


## ScienceConsumer
`node_name = teleop_science`

websocket: `science`

### Forwarding
- `/led`
- `/science_thermistors`
- `/science_heater_state`
- `/science_oxygen_data`
- `/science_methane_data`
- `/science_uv_data`
- `/science_temperature_data`
- `/science_humidity_data`
- `/sa_controller_state`
- `/sa_gear_diff_position`

### Services
- `/science_change_heater_auto_shutoff_state`
- `/sa_enable_limit_switch_sensor_actuator`
- `/sa_gear_diff_set_position`
- `/science_enable_heater_<name>` (one per `name` in `heater_names`)
- `/science_enable_white_led_a`
- `/science_enable_white_led_b`

### Receiving
- `heater_enable`
- `set_gear_diff_pos`
- `auto_shutoff`
- `white_leds`
- `ls_toggle`


---


## WaypointsConsumer
`node_name = teleop_waypoints`

websocket: `waypoints`

### Receiving
- `save_auton_waypoint_list`
- `save_basic_waypoint_list`
- `save_current_auton_course`
- `save_current_basic_course`
- `delete_auton_waypoint_from_course`
- `get_basic_waypoint_list`
- `get_auton_waypoint_list`
- `get_current_basic_course`
- `get_current_auton_course`


---
