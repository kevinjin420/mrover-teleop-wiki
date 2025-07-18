# Three.js urdf joint transforms

## arm.urdf

### arm_a

name: `base_link_joint`

connects: `base_link`, `a_link`

relative: `xyz="16.4882 20.0235 49.1172"`

absolute: `xyz="16.4882 20.0235 49.1172"`

### arm_b

name: `a_joint`

connects: `a_link`, `b_link`

relative: `xyz="3.4348 -0.0005 4.9028"`

absolute: `xyz="19.923 20.023 54.02"`

### arm_c

name: `b_joint`

connects: `b_link`, `c_link`

relative: `xyz="53.4363 0.0005 0.9052"`

absolute: `xyz="73.3593 20.0235 54.9252"`

### arm_d

name: `c_joint`

connects: `c_link`, `d_link`

relative: `xyz="54.6037 0 8.8594"`

absolute: `xyz="127.963 20.0235 63.7846"`

### arm_e

name: `d_joint`

connects: `d_link`, `e_link`

relative: `xyz="4.488 0 0"`

absolute: `xyz="132.451 20.0235 63.7846"`

---

### arm_gripper

name: `gripper_link`

connects: `e_link`, `gripper_link`

relative: `rpy="1.570796 0 0" xyz="0.0875 0 0"` (positive rotation by pi/2 on x-axis)

absolute: `xyz="132.451 20.0235 63.7846"`


## rover.urdf

