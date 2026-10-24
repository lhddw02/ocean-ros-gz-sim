# Floating-station mooring system

This Gazebo Harmonic (`gz-sim8`) system applies a horizontal spring and damper
to a dynamic floating hull, with a separate yaw spring and damper. It adds its
wrench to the hull's hydrodynamic force each physics step. It applies no
vertical force and no roll/pitch restoring spring, so the hull can heave and tilt
with the ocean. Optional roll/pitch damping reduces rapid rocking.

The mooring is a linear spring approximation, not a cable or seabed simulation.
Put the selected link's origin at its center of mass to avoid an unintended
moment from the horizontal force. A broad buoyant hull supplies hydrostatic
roll/pitch stability. The mooring does not replace that buoyancy model.

```xml
<plugin filename="libocean-station-mooring-system.so"
        name="ocean_station_systems::StationMooring">
  <link_name>hull</link_name>
  <horizontal_stiffness>600</horizontal_stiffness>
  <horizontal_damping>900</horizontal_damping>
  <max_horizontal_force>5000</max_horizontal_force>
  <yaw_stiffness>200</yaw_stiffness>
  <yaw_damping>300</yaw_damping>
  <max_yaw_torque>1000</max_yaw_torque>
  <roll_pitch_damping>0</roll_pitch_damping>
  <max_roll_pitch_torque>1000</max_roll_pitch_torque>
</plugin>
```

These values are defaults. Horizontal stiffness is in N/m, linear damping in
N s/m, yaw stiffness in N m/rad, angular damping in N m s/rad, force limits in N,
and torque limits in N m. All gains and limits must be finite and nonnegative;
zero limits disable that force or torque. Horizontal and roll/pitch limits cap
the vector magnitude, rather than each axis separately.

The anchor defaults to the link's initial world x/y position and yaw. Optional
`anchor_x`, `anchor_y` (metres) and `anchor_yaw` (radians) override individual
coordinates. If `link_name` is omitted, the model's canonical link is used.
For a hull of mass `m`, a horizontal damping near `2 * sqrt(stiffness * m)` is
a useful starting point; retune after changing the hull mass or wave conditions.

Build this package with `ament_cmake`, `libgz-sim8-dev` and `libgz-plugin2-dev`.
The installed library is `lib/libocean-station-mooring-system.so`. Add the
workspace's installed `lib` directory to `GZ_SIM_SYSTEM_PLUGIN_PATH`.

With tests enabled, `colcon test --packages-select ocean_station_systems` checks
restoring signs, force/torque saturation and the shortest yaw path across the
angle wrap. The Gazebo API contracts used here are documented in the
[gz-sim8 Link API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1Link.html).

## Contact sensing after world reset

The landing world also loads `libocean-replay-contact-system.so`, alias
`ocean_station_systems::ReplayContact`, at world level in place of the native
Contact system:

```xml
<plugin filename="libocean-replay-contact-system.so"
        name="ocean_station_systems::ReplayContact"/>
```

This system adapts Gazebo's Apache-2.0
[Contact implementation](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/contact/Contact.cc).
The native Harmonic system discovers sensors through `EachNew` only. When a
world reset reconstructs it, that iteration's new-entity flags are cleared
before its next update, leaving its contact sensor list empty. `ReplayContact`
scans all sensors on its first update and implements `ISystemReset` to rebuild
their publishers and restore collision `ContactSensorData` components. It
discards pending contacts and timestamps from the previous run.

Scoped topic names, collision names and the nested `<contact><topic>` override
match the native system. It publishes only positive contacts. The existing four
foot-contact bridge mappings therefore work before and after replay. Load one
contact system per world. The contact plugin additionally depends on
`libgz-transport13-dev` and `libgz-msgs10-dev`; the Docker image supplies them.

With tests enabled, `contact_reset_test` drops a box onto a plane and requires
real named collision messages on the default scoped topic and an explicit
topic override, both before and after two complete world resets. The test
retains a small JSON report and Gazebo log in the package's build directory.
