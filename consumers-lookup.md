Consumers handle communication to ROS2 topics and services, as well as frontend websockets

With the 25' school year, we have switched from one main consumers.py file to multiple consumers to increase the bandwidth and reduce latency to the frontend. 

Each consumer corresponds to a core function of the rover, such as the robotic arm, autonomous navigation, etc. 

---

`!` = warning, red squiggle

```
AutonTask
├── OdometryReading !
│   ├── FlightAltitudeIndicator !
│   └── IMUCalibration
├── CameraFeed !
├── AutonRoverMap !
├── AutonWaypointEditor !
│   ├── AutonWaypointItem !
│   ├── AutonModeCheckbox
│   ├── BasicCheckbox
│   ├── VelocityReading
│   └── AutonWaypointStore !
├── DriveControls
├── MastGimbalControls
└── ControllerDataTable

CameraView
├── ToggleButton
└── CameraFeed !

DMESTask !
├── BasicRoverMap !
├── BasicWaypointEditor !
│   └── BasicWaypointItem
├── DriveControls
├── ArmControls
├── Rover3D ! (separate back out to a js file, then @ts-ignore)
├── ControllerDataTable
└── MastGimbalControls

ISHTask
├── SelectSite ! (no warning but written like shit)
├── NinhydrinBenedict
├── CameraFeed !
├── ToggleButton
├── AutoShutdown
│   └── ToggleButton
├── SensorData ! (mess)
└── WhiteLEDs

SATask
├── BasicRoverMap !
├── SoilData ! (this component does nothing??)
├── BasicWaypointEditor !
│   └── BasicWaypointItem
├── DriveControls
├── MastGimbalControls
├── OdometryReading !
│   ├── FlightAltitudeIndicator !
│   └── IMUCalibration
├── ControllerDataTable
├── SAArmControls
├── HexHub ! (fucking dogshit)
└── LSActuator ! (maybe rethink)

Unused:
CacheControls
CalibrationCheckbox

Pano not implemented, SHOULD GO IN MAST

```


ROVER3D FK AND IK NOT USED

DMS waypoints not properly typed, errors everywhere

Redo checkboxes, ugly as fuck

combine ish selectsite, autoshutdown, and white leds into one component

navbar not typesafe

hexhub and actuator are a disaster

SA and Auton tasks Controllerdatatable is deprecated???

Nothing in SoilData seems to work? double check

SOIL DATA CSV STILL TAKES A SCREENSHOT FUCK

---

deprecated:

orientation

soil_temp

soil_humidity

poly_fit


---

# Websocket Usage per Mission
Check which websockets a certain mission uses

### Auton
- `auton`
- `drive`
- `nav`
- `science`
- `waypoints`

### DM/ES
- `arm`
- `drive`
- `mast`
- `nav`
- `waypoints`

### ISH
- `science`

### SA
- `arm`
- `mast`
- `nav`
- `science`
- `waypoints`


---

# Topic and Service consumer lookup
Check below to see which consumer handles a topic or service

## ArmConsumer

websocket: `arm`

### Publishing
- `arm_throttle_cmd`
- `ee_pos_cmd`
- `ee_vel_cmd`
- `controller_cmd_vel`
- `sa_throttle_cmd`

### Forwarding
- `arm_controller_state` => `arm_state`
- `sa_controller_state` => `sa_state`
- `arm_joint_data` => `fk` (integrates with rover3d, not yet impl.)

### Receiving
- `ra_controller`
- `ra_mode`
- `sa_controller`
- `sa_mode`

---


## AutonConsumer

websocket: `auton`

### Services
- `enable_teleop`
- `enable_auton`

### Receiving
- `teleop_enable`
- `auton_enable`

---


## DriveConsumer

websocket: `drive`

### Publishing
- `joystick_cmd_vel`

### Forwarding
- `drive_left_controller_data` => `drive_left_state`
- `drive_right_controller_data` => `drive_right_state`
- `drive_controller_data` => `drive_state`

### Receiving
- `joystick`

---


## MastConsumer

websocket: `mast`

### Publishing
- `mast_gimbal_throttle_cmd`

### Receiving
- `mast_keyboard`

---


## NavConsumer

websocket: `nav`

### Forwarding
- `nav_state` => `nav_state`
- `gps/fix` => `gps_fix`
- `basestation/position` => `basestation_position`
- `drone_odometry` => `drone_waypoint`

---


## ScienceConsumer

websocket: `science`

### Forwarding
- `led` => `led`
- `science_thermistors` => `thermistors`
- `science_heater_state` => `heater_states`
- `science_oxygen_data` => `oxygen`
- `science_uv_data` => `uv`
- `science_temperature_data` => `temperature`
- `science_humidity_data` => `humidity`
- `science_methane_data` => `methane` (cannot find use)
- `sa_gear_diff_position` => `hexhub_site` (cannot find use)

### Services
- `science_change_heater_auto_shutoff_state`
- `sa_enable_limit_switch_sensor_actuator`
- `sa_gear_diff_set_position`
- `science_enable_heater_<name>` (one per `name` in `heater_names`)
- `science_enable_white_led_a`
- `science_enable_white_led_b`

### Receiving
- `heater_enable`
- `set_gear_diff_pos`
- `auto_shutoff`
- `white_leds`
- `ls_toggle`


---


## WaypointsConsumer

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
