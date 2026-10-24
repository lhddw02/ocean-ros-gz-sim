# Landing replay control

`LandingReplay` is a Gazebo GUI 8 plugin. Its **Replay landing** button sends a
single `gz.msgs.Boolean` (`data: true`) to `/landing/replay`. The ROS coordinator
resets and rearms the scene while the container and GUI stay open.

The plugin listens on `/landing/replay_status` (`gz.msgs.StringMsg`) for `starting`,
`ready`, `resetting`, `arming`, `resuming`, `replaying`, `landed`, or
`error: <detail>`. The button disables during
the request and reset, then becomes available again during flight. The ROS bridge
and coordinator provide these topics; the widget does not reset the world itself.

The library installs to `lib/libLandingReplay.so` with its QML embedded in the
library. Add the install `lib` directory to `GZ_GUI_PLUGIN_PATH` and load
`<plugin filename="LandingReplay" name="Landing replay"/>` in the landing world's
GUI configuration. Optional `<topic>` and `<status_topic>` override the topics.

With `BUILD_TESTING=ON`, `replay_button_test` creates the real QML button in a
software-rendered Qt view, clicks it, and verifies the Gazebo Transport message
and reset acknowledgement interlock.
