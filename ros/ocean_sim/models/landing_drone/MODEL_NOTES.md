# Landing drone model

The quadrotor uses original, connected Collada visual meshes, licensed
GPL-3.0-or-later with this ROS package. No external vehicle mesh or runtime
Gazebo Fuel download is required. Every visible component is a mesh: the
chamfered orange canopy, carbon X frame, arm saddles, service panel, aluminum
hardware, supported motor bells and shafts, shaped propellers, landing struts,
contact feet, camera housing and optical glass, and navigation lights.

The structural arms overlap both the body and motor seats. Each long landing
strut starts inside the shell at X = +/-0.106 m, Y = +/-0.069 m, Z = -0.025 m,
passes through a connected knee and brace, and ends inside its existing
contact foot at X/Y = +/-0.300 m, Z = -0.425 m. The motor seats extend from
body Z = -0.012 m to +0.027 m and overlap the fixed stator supports. Rotating
motor bells begin at body Z = +0.034 m; their shafts connect to the propeller
hubs at body Z = +0.088 m. The shaped blades sit above the bells instead of
cutting through them. Colored tip marks overlap the actual blade surfaces
and turn with their rotor links.

The camera housing stops above the optical center. Its thin glass surface is
at body Z = -0.0995 m, behind the unchanged downward optical origin at
Z = -0.100 m. It therefore cannot block the camera's forward view. The
struts spread outward and preserve a clear central view of the landing tag.

Regenerate the assets from the project root using
`python scripts/generate-drone-meshes.py`. This uses only the Python standard
library and produces metre-scale, Z-up Collada files with explicit normals.
`meshes/geometry_manifest.json` records the bounds and closed, outward-facing
triangles of every solid. The generator compares the complete nonvisual SDF
before and after replacing visuals, and checks it against the saved original
fingerprint. Link poses, masses, inertias, collision shapes, joints, sensors,
camera calibration and controller parameters are preserved.

The motor constants and controller gain configuration follow Gazebo Sim's
[multicopter velocity-control example](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/multicopter_velocity_control.sdf).
That example and the underlying systems are Copyright Open Source Robotics
Foundation and licensed under
[Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0).
The model geometry, rotor naming, command topics, camera, legs and contact
sensors were created for this ocean-landing demonstration.

The vehicle has four real revolute rotor links. Gazebo's
`MulticopterVelocityControl` computes motor speeds using its Lee velocity
controller, and four `MulticopterMotorModel` instances apply thrust and moments.
It is a simple velocity-controlled dynamics demonstration; it does not include
a PX4 or ArduPilot flight stack. The low-level Gazebo controller uses simulated
body velocity and attitude internally. The separate AprilTag landing controller
uses rendered camera measurements for target-relative guidance.

| Property | Value |
| --- | --- |
| Total mass | 1.20 kg (1.16 kg body plus four 0.01 kg rotors) |
| Motor positions | X and Y = +/-0.23 m |
| Approximate rotor-to-rotor span | 0.72 m including propellers |
| Body collision | 0.26 x 0.18 x 0.10 m |
| Landing foot centers | X and Y = +/-0.30 m; Z = -0.425 m |
| Lowest landing-foot point | Body Z = -0.450 m |
| Camera center | Body Z = -0.100 m |
| Camera clearance above lowest feet | 0.350 m |
| Camera image | 960 x 720 RGB, 20 Hz, horizontal FOV 1.8 rad |
| Camera mount rotation | Roll 0, pitch +pi/2, yaw 0 |

The camera faces down: optical right is body -Y, optical down is body -X,
and optical forward is body -Z. It uses an ordinary perspective projection;
intrinsics come from the actual camera-info stream. The wide field and long
landing legs keep the complete 0.40 m target visible close to touchdown.

| Gazebo topic | Message / use |
| --- | --- |
| `/drone/cmd_vel` | `gz.msgs.Twist`, linear velocities in body axes and yaw rate |
| `/drone/enable` | `gz.msgs.Boolean`, enables/disables the motor controller |
| `/drone/camera/image_raw` | `gz.msgs.Image`, actual rendered downward RGB view |
| `/drone/camera/camera_info` | `gz.msgs.CameraInfo`, actual projection intrinsics |
| `/drone/pose` | `gz.msgs.Pose`, 30 Hz validation telemetry |

The four Gazebo foot-contact topics are:

```text
/world/ocean/model/landing_drone/link/base_link/sensor/landing_contact_fl/contact
/world/ocean/model/landing_drone/link/base_link/sensor/landing_contact_fr/contact
/world/ocean/model/landing_drone/link/base_link/sensor/landing_contact_rl/contact
/world/ocean/model/landing_drone/link/base_link/sensor/landing_contact_rr/contact
```

`config/landing_bridge.yaml` combines these into the single ROS topic
`/drone/contacts`, of type `ros_gz_interfaces/msg/Contacts`. The controller and
verification probe subscribe to that ROS topic. Harmonic's
[Contact system](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/contact/Contact.cc)
uses the sensor's scoped topic unless an override is placed inside
`<contact><topic>...</topic></contact>`; the outer `<sensor><topic>` does not
override it. Contact messages are published when contact points exist.

Foot collision names are `foot_fl`, `foot_fr`, `foot_rl`, and `foot_rr`, on
`landing_drone::base_link`. The station collision is
`floating_station::base_link::hull_collision`. Contact guidance must filter
for a foot touching that station collision. A camera image alone is not proof
of physical touchdown.

Gazebo Physics includes those scoped collision names in each contact by
default, and the ROS bridge preserves them. If configuring the Physics plugin,
keep `<contacts><include_entity_names>true</include_entity_names></contacts>`
enabled so the touchdown filters can identify the physical pair.

The world must load Physics, Sensors (Ogre2), and Contact systems. The drone
starts airborne and needs a first Twist command promptly: the low-level
controller waits for that command before spinning its rotors. To maintain
flight, keep it enabled and send zero velocity to hover. Disable it only after
supported contact, because disabling it while airborne removes motor thrust.
