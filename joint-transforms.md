# Three.js urdf joint transforms

## arm_a

name: `base_link_joint`

connects: `base_link`, `a_link`

relative: `xyz="16.4882 20.0235 49.1172"`

absolute: `xyz="16.4882 20.0235 49.1172"`

## arm_b

name: `a_joint`

connects: `a_link`, `b_link`

relative: `xyz="3.4348 -0.0005 4.9028"`

absolute: `xyz="19.923 20.023 54.02"`

## arm_c

name: `b_joint`

connects: `a_link`, `b_link`

relative: `xyz="53.4363 0.0005 0.9052"`

absolute: `xyz="73.3593 20.0235 54.9252"`

## arm_d

name: `c_joint`

connects: `a_link`, `b_link`

relative: `xyz="54.6037 0 8.8594"`

absolute: `xyz="127.963 20.0235 63.7846"`

## arm_e

name: `d_joint`

connects: `a_link`, `b_link`

relative: `xyz="4.488 0 0"`

absolute: `xyz="132.451 20.0235 63.7846"`

---

`arm.urdf`:

```xml
  <joint name="base_link_joint" type="prismatic">
    <!-- <origin rpy="0 0 0" xyz="0 0 0"/> -->
    <origin rpy="0 0 0" xyz="16.4882 20.0235 49.1172"/>
    <!-- y -10 not applied yet -->
    <parent link="base_link"/>
    <child link="a_link"/>
    <axis xyz="0 1 0"/>
    <!-- effort in N-m, velocity in rad/s -->
    <limit effort="30" lower="-40" upper="0" velocity="1"/>
    <!-- damping in N-m-s/rad -->
    <dynamics damping="1.0"/>
  </joint>
  <joint name="a_joint" type="revolute">
    <!-- <origin rpy="0 0 0" xyz="0 0 0"/> -->
    <origin rpy="0 0 0" xyz="3.4348 -0.0005 4.9028"/>
    <!-- <origin rpy="0 0 0" xyz="19.923 20.023 54.02"/> -->
    <parent link="a_link"/>
    <child link="b_link"/>
    <axis xyz="0 1 0"/>
    <!-- effort in N-m, velocity in rad/s -->
    <limit effort="30" lower="-1.570796" upper="0" velocity="1"/>
    <!-- damping in N-m-s/rad -->
    <dynamics damping="1.0"/>
  </joint>
  <joint name="b_joint" type="revolute">
    <!-- <origin rpy="0 0 0" xyz="0 0 0"/> -->
    <origin rpy="0 0 0" xyz="53.4363 0.0005 0.9052"/>
    <!-- <origin rpy="0 0 0" xyz="73.3593 20.0235 54.9252"/> -->
    <parent link="b_link"/>
    <child link="c_link"/>
    <axis xyz="0 1 0"/>
    <!-- effort in N-m, velocity in rad/s -->
    <limit effort="30" lower="0" upper="3.141592" velocity="1"/>
    <!-- damping in N-m-s/rad -->
    <dynamics damping="1.0"/>
  </joint>
  <joint name="c_joint" type="revolute">
    <!-- <origin rpy="0 0 0" xyz="0 0 0"/> -->
    <origin rpy="0 0 0" xyz="54.6037 0 8.8594"/>
    <!-- <origin rpy="0 0 0" xyz="127.963 20.0235 63.7846"/> -->
    <parent link="c_link"/>
    <child link="d_link"/>
    <axis xyz="0 1 0"/>
    <!-- effort in N-m, velocity in rad/s -->
    <limit effort="30" lower="-3.141592" upper="3.141592" velocity="1"/>
    <!-- damping in N-m-s/rad -->
    <dynamics damping="1.0"/>
  </joint>
  <joint name="d_joint" type="revolute">
    <!-- <origin rpy="0 0 0" xyz="0 0 0"/> -->
    <origin rpy="0 0 0" xyz="4.488 0 0"/>
    <!-- <origin rpy="0 0 0" xyz="132.451 20.0235 63.7846"/> -->
    <parent link="d_link"/>
    <child link="e_link"/>
    <axis xyz="1 0 0"/>
    <!-- effort in N-m, velocity in rad/s -->
    <limit effort="30" lower="-3.141592" upper="3.141592" velocity="1"/>
    <!-- damping in N-m-s/rad -->
    <dynamics damping="1.0"/>
  </joint>
```
