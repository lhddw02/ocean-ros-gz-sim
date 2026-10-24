# Ocean simulation workflow

Choose the scene from your Ubuntu project directory:

| Scene | Command |
| --- | --- |
| Ocean waves, duck and buoy, without a drone or landing station | `bash scripts/run-wsl.sh start` |
| Floating station and drone with live AprilTag camera panel | `bash scripts/run-wsl.sh landing` |

Both commands recreate `ocean-sim` and reset the world. The landing scene opens a
**Downward camera - AprilTag** panel in Gazebo with the detected tag, relative
pose information, landing state and motor state. Its annotated view updates at
up to 5 Hz while the detector uses the full camera stream. See
[LANDING.md](LANDING.md) for the continuous side-view GIF, hover mode and
landing-specific commands. Rebuild once with `bash scripts/run-wsl.sh build`
when upgrading an older image to include the camera panel and replay controls.

In the landing scene, click **Replay landing** in the upper-left **Landing demo**
card for another flight. It resets time and all models, then enables landing;
the current view and Docker container remain open. It works while flying,
paused or landed. The button is temporarily disabled during reset and arming.
The selected tag stays the same; rerun `landing` when changing its configuration.
[Replay commands and status](LANDING.md#replay-the-landing-in-gazebo).

Use [README.md](README.md) for Windows/WSL2 setup and the first image build. Run the following commands in **Ubuntu on WSL2**, from the tested project at `/mnt/c/Users/aston/Documents/Codex/2026-10-11/i-x20/outputs/ocean-ros-humble`. Keep Docker Desktop running. The README also describes an optional copy to `~/simulations/ocean-ros-humble` for faster builds. If you move it, update the `cd` paths below and edit that Linux copy consistently; access it from Windows through `\\wsl.localhost\Ubuntu-22.04\home\YOUR_LINUX_USER\simulations\ocean-ros-humble`.

**Status, 23 October 2026:** WSL2 and Docker work, the patched image is built, and the Gazebo GUI runs with visibly animated waves. The GPU check and actual GUI Ogre2 log confirm the RTX 3060 through D3D12, OpenGL 4.2 and Mesa 23.2.1. The rebuilt image passed the headless smoke test with 8.00 seconds of advancing clock and 0.526 m of buoy-height variation. A separate probe in the actual GUI simulation measured 8.00 seconds and 0.557 m. See [VALIDATION.md](VALIDATION.md) and the [actual GUI recording](verification/gazebo-ocean.gif).

## Accept the first working environment

Complete these gates before adding a vessel or controller:

1. `bash scripts/run-wsl.sh doctor` passes the WSL, Docker and display prerequisites.
2. `bash scripts/run-wsl.sh build` completes.
3. `bash scripts/run-wsl.sh check` reports accelerated OpenGL, a D3D12 NVIDIA RTX 3060 renderer, and OpenGL 4.2 or newer.
4. `bash scripts/run-wsl.sh smoke` passes its clock and buoy-motion checks.
5. `bash scripts/run-wsl.sh start` opens the ocean window; the surface and buoy move, and the real-time factor appears in World Stats.
6. A sourced container shell sees `/clock` and `/ocean/buoy/pose`; a short bag contains messages after you stop recording.

`check` validates accelerated OpenGL, `smoke` validates headless physics and the bridge, and `start` plus GUI inspection validates the ocean material and visible wave animation. Keep the GPU check result and `/root/.gz/rendering/ogre2.log` from the running container as rendering evidence; `nvidia-smi` alone does not validate this OpenGL path.

If upgrading an older image with the Ogre2 texture assertion, update the project files and run:

```bash
bash scripts/run-wsl.sh build
bash scripts/run-wsl.sh check
bash scripts/run-wsl.sh smoke
bash scripts/run-wsl.sh restart
```

The build applies [asv-wave-visual-dedup.patch](patches/asv-wave-visual-dedup.patch) to pinned ASV commit `ca8629df4e191235753dfae92ef725d30b923364`. It prevents duplicate WavesVisual initialization for the same ECM/entity, which previously tried to resize resident Ogre2 textures. Keep `GzSceneManager` in the world. This is a compiled-plugin fix: the development overlay does not replace the rebuild. [Gazebo issue #1647](https://github.com/gazebosim/gz-sim/issues/1647) documents the related repeated visual-plugin initialization.

The following [asv-wave-reset.patch](patches/asv-wave-reset.patch) adds Gazebo
reset support to preserve the wave visual's GPU resources when the replay
button rewinds the world. Upgrading this compiled plugin also requires a build.

## Daily routine: three Ubuntu terminals

### Terminal 1: run Gazebo and follow its logs

```bash
cd /mnt/c/Users/aston/Documents/Codex/2026-10-11/i-x20/outputs/ocean-ros-humble
bash scripts/run-wsl.sh start
bash scripts/run-wsl.sh logs
```

The simulation runs in the background. This scene contains ocean waves, the duck and the buoy. To watch a drone landing with the live camera panel, run `bash scripts/run-wsl.sh landing` instead of `start`, then use **Replay landing** for subsequent flights. `Ctrl+C` here stops following logs; it does not stop Gazebo. Pause/play in the Gazebo window when inspecting a scene.

### Terminal 2: inspect ROS and change the waves

```bash
cd /mnt/c/Users/aston/Documents/Codex/2026-10-11/i-x20/outputs/ocean-ros-humble
bash scripts/run-wsl.sh exec
ros2 topic list
ros2 topic echo /clock --once
ros2 topic echo /ocean/buoy/pose --once
ros2 topic hz /ocean/buoy/pose
```

`/ocean/buoy/pose` is a dedicated `geometry_msgs/msg/PoseStamped` stream for the spherical buoy. Watch `pose.position.z` to inspect heave. The retained `/ocean/poses` raw SceneBroadcaster stream has empty entity frame names in this configuration; use the dedicated topic instead of filtering that stream by entity name. Neither stream supplies a complete robot TF tree.

Use `Ctrl+C` to stop the topic-rate display, then try one wave setting at a time:

```bash
set-waves 2 135 0.5
set-waves 5 135 1
set-waves 8 135 1
```

These are progressively stronger test inputs: wind speed in m/s, angle in degrees, and steepness. They are **not calibrated Beaufort or significant-wave-height classes**. The command changes both wave plugins through Gazebo transport; it does not edit the SDF file. Restarting the simulation restores the file's defaults. Keep one setting for each recorded experiment and let the vessel settle before comparing results.

### Terminal 3: record and inspect a short experiment

```bash
cd /mnt/c/Users/aston/Documents/Codex/2026-10-11/i-x20/outputs/ocean-ros-humble
bash scripts/run-wsl.sh exec
set-waves 5 135 1
```

This selects the 5 m/s baseline used in the recording name. Let the scene settle, then record:

```bash
run_id="wind5_$(date +%Y%m%d_%H%M%S)"
cp /opt/ocean/build-info.txt "/recordings/${run_id}-build-info.txt"
ros2 bag record --use-sim-time -o "/recordings/$run_id" /clock /ocean/buoy/pose
```

Record for about 30 seconds of simulation time, then press `Ctrl+C` to finalize the bag. Still in the container:

```bash
ros2 bag info "/recordings/$run_id"
exit
```

The recording is already saved in your Ubuntu project's `recordings/` directory through a bind mount. Back in Ubuntu:

```bash
ls recordings/
bash scripts/run-wsl.sh stop
```

Use a fresh output name for each bag and back up `recordings/` alongside your source files. `--use-sim-time` uses the incoming `/clock` for record timestamps and waits for its first message. A paused simulation does not advance simulation time. Start with the two explicit topics rather than recording every future camera stream. See the [Humble rosbag2 documentation](https://github.com/ros2/rosbag2/blob/humble/README.md).

The container runs as root, so files it creates in mounted directories may be owned by root. They are usually readable from Ubuntu; if you need to edit them as your Ubuntu user, stop recording and use `sudo chown -R "$(id -u):$(id -g)" recordings` from the project directory.

## What survives stopping and what needs a rebuild

| Change or output | Persistence | Action |
| --- | --- | --- |
| Files edited in your Ubuntu project directory | Survive stopping/removing containers | Keep this directory as the source of truth |
| `set-waves` changes | Live session only | Apply again or edit both SDF `<wave>` blocks |
| Bags written under `/recordings` | Host `recordings/` directory; survives container removal | Finalize with `Ctrl+C`, then back up |
| Source/build files under `/workspace` | Host `workspace/` directory; survives container removal | Create source files as your Ubuntu user where practical |
| Files elsewhere inside `ocean-sim` | Container only | Copy needed files out before `run-wsl.sh stop` |
| World, model, bridge YAML or launch edits under host `ros/ocean_sim` | Host edits survive; standard mode uses image copies | Standard mode: build then restart; dev mode: restart |
| Dockerfile packages, upstream version or container scripts | Included at image build | Rebuild image, then recreate container |
| Compose GPU/display/environment settings | Compose file survives | Stop and start to recreate with new settings |

For the standard image-based workflow:

```bash
bash scripts/run-wsl.sh build
bash scripts/run-wsl.sh restart
```

`start` does not build the image. The runner's `restart` recreates the ocean-only scene so a newly built image is used. For a fresh landing after a rebuild, use `bash scripts/run-wsl.sh landing`. `docker restart ocean-sim` only restarts that same container; it does not pick up a new image or new mounts. A diagnostic `shell` creates a temporary container removed on exit; keep its work under `/workspace` or `/recordings`.

For fast edits to the world, model, bridge and launch files, use the optional development overlay:

```bash
bash scripts/run-wsl.sh stop
OCEAN_DEV=1 bash scripts/run-wsl.sh start
# Edit host ros/ocean_sim files, then reload:
OCEAN_DEV=1 bash scripts/run-wsl.sh restart
```

This mounts `ros/ocean_sim` over the installed package's **share directory** read-only. SDF/config/launch edits take effect on a fresh launch without an image build. Keep `OCEAN_DEV=1` on subsequent runner calls for the same mode. Package dependencies, Python modules and ASV plugin changes still need a build. To return to the image's packaged files, stop, then start without `OCEAN_DEV=1`.

Do not mount an empty folder over `/opt/ocean_ws/install` or `/opt/asv_ws/install`: that hides the working installation. [Docker bind mounts](https://docs.docker.com/engine/storage/bind-mounts/) share these persistent host files with containers.

## Add a vessel, sensors and a controller in stages

1. **Passive vessel:** put a uniquely named SDF model in `ros/ocean_sim/models/YOUR_VESSEL`, with `model.config`, appropriate mass/inertia and a closed hull collision mesh. Include it in `worlds/ocean.sdf`. Use the local `models/spherical_buoy/model.sdf` as a working hydrodynamics and pose-publication example; adapt its entity names, displacement and drag parameters to your hull. Check flotation and roll/pitch response before propulsion. The upstream plugin can target a model, link or collision through `<enable>`; see [ASV Wave Sim](https://github.com/srmainwaring/asv_wave_sim#hydrodynamics-plugin).
2. **Sensors:** add one sensor at a time and its required Gazebo system plugin. Inspect its Gazebo topic with `gz topic -l`, then add a supported message mapping to `config/bridge.yaml`. Use `GZ_TO_ROS` for telemetry and `ROS_TO_GZ` for actuator commands. The supplied bridge only exposes clock and poses; it supplies no vessel command topic. Follow the [Harmonic sensor tutorial](https://gazebosim.org/docs/harmonic/sensors/) and [ROS integration tutorial](https://gazebosim.org/docs/harmonic/ros2_integration/).
3. **Propulsion and controller:** first verify that the Gazebo actuator responds to a known command. Put your ROS controller package in host `workspace/src`, which appears as `/workspace/src` in the container. Build and source its overlay before running it in `ocean-sim`; use namespaced topics for each vessel. Add a dedicated pose publisher and bridge for the vessel, following the buoy example. The raw `/ocean/poses` stream lacks entity frame names here; provide the TF tree or odometry interface required by your controller.
4. **Tests:** establish flotation in gentle waves, then command tracking, then sensors, then stronger winds. Record commands, telemetry, clock and the controller's errors; compare one changed variable at a time.

For example, after creating `workspace/src/YOUR_PACKAGE` on the Ubuntu host:

```bash
bash scripts/run-wsl.sh exec
cd /workspace
colcon build --symlink-install
source install/setup.bash
ros2 run YOUR_PACKAGE YOUR_EXECUTABLE --ros-args -p use_sim_time:=true
```

Replace the two uppercase names with your package and executable. Source `/workspace/install/setup.bash` in each new shell that runs your package. Install extra dependencies through the Dockerfile for repeatable builds.

ROS discovery is simplest when all nodes run inside `ocean-sim`. `run-wsl.sh exec` joins that container; `run-wsl.sh shell` creates a separate container. External ROS nodes require compatible DDS networking, the same `ROS_DOMAIN_ID` (default 30), and appropriate firewall/discovery settings. Publishing one ordinary Docker port does not solve DDS discovery. See [Docker networking](https://docs.docker.com/engine/network/). Gazebo transport also needs a matching `GZ_PARTITION` when processes are separated.

## Repeat experiments and tune performance

Keep the bag, your world/model/bridge files, image ID (`docker image inspect --format '{{.Id}}' ros-humble-ocean:harmonic`), `/opt/ocean/build-info.txt`, wave values, initial vessel pose, settling time and recording duration for each run. Restart between baseline ocean runs so all start from the SDF initial state. For the landing demo, **Replay landing** restores all model poses without a container restart. Finish the current bag first because replay returns `/clock` to zero. A bag replays ROS messages; it does not reconstruct or deterministically replay the ocean physics. Exact FFT realization repeatability needs separate seed/control validation.

For analysis replay, stop live Gazebo first, then open the temporary sourced shell:

```bash
bash scripts/run-wsl.sh stop
bash scripts/run-wsl.sh shell
ros2 bag info /recordings/YOUR_BAG_DIRECTORY
ros2 bag play /recordings/YOUR_BAG_DIRECTORY
```

Replace `YOUR_BAG_DIRECTORY` with the saved bag's directory name. With `/clock` in the bag, replay that recorded topic as the single clock source; do not also start live Gazebo in the same ROS domain. Start analysis nodes in this same temporary container with `docker exec` when you need to avoid extra DDS network setup.

Start with the existing 128×128 FFT grid, 100 m tile and simple buoy. Check World Stats: a sustained real-time factor below 1 means the simulation runs slower than wall time. On your RTX 3060, try a smaller GUI window, fewer camera pixels/lower sensor rates, fewer visible tiles and simpler hull collisions. If CPU load limits physics, try a 64×64 FFT grid in **both** wave blocks, retaining the current tile/mesh size. FFT grid dimensions must remain powers of two. These changes trade fidelity for speed; revalidate buoy motion. Change the physics step size only after checking the accuracy your controller needs.

Keep the adapter check enabled. The GPU accelerates rendering and rendering sensors; the wave FFT and hydrodynamic workload still depend on CPU capacity. Increase WSL memory only when builds or runtime show memory pressure; an RTX 3060 alone does not determine simulation speed.
