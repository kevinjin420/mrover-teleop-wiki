upgraded all bun packages to most recent

SASS deprecation warning has not been fixed for a year: [issue](https://github.com/twbs/bootstrap/issues/40849)

in package.json and bun.lock, keep sass on 1.77.6, everything else can be upgraded to most recent

should this be written somewhere for the next lead?

---
Unused Components: 

CacheControls

CalibrationCheckbox

SoilData (no data feed)

PDBFuse


---

ROVER3D IK NOT USED, FK ~~NOT IMPLEMENTED~~ DONE

DMS waypoints not properly typed, errors everywhere

Redo checkboxes, ugly

combine ish selectsite, autoshutdown, and white leds into one component

hexhub and actuator are a disaster

Nothing in SoilData seems to work? double check

SOIL DATA CSV STILL TAKES A SCREENSHOT FUCK

methane removed altogether? pretty sure

auton task stuck_status: is it ever changed????

websockets sometimes crash, slow reload

auton ws errors and has to be restarted, adds ~2s

sim not up to date - joint DE default orientation has changed

sim not updated to handle cam and gripper

future point cloud impl? on which task?

sa arm controls broken, missing field "site"

improve button readability, make text black in certain low contrast scenarios

ask john for any more ui complaints

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