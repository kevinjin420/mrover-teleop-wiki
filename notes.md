
Unused Components: 

CacheControls

CalibrationCheckbox

SoilData (no data feed)

PDBFuse


---

ROVER3D IK NOT USED, FK NOT IMPLEMENTED

DMS waypoints not properly typed, errors everywhere

Redo checkboxes, ugly as fuck

combine ish selectsite, autoshutdown, and white leds into one component

hexhub and actuator are a disaster

Nothing in SoilData seems to work? double check

SOIL DATA CSV STILL TAKES A SCREENSHOT FUCK

methane removed altogether?

---

deprecated (old consumers.py did not handle):

orientation

soil_temp

soil_humidity

poly_fit

pano_feedback (originally in CameraView.vue, removed)

---


`!` = has warnings, red squiggle

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


```