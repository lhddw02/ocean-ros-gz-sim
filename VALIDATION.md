# Validation status — 24 October 2026

Ubuntu-22.04 and Docker Desktop are running on WSL2. The Linux Docker engine is reachable (29.9.2; Ubuntu Compose 5.6.1). The local `ros-humble-ocean:harmonic` image has been rebuilt with the smoke-test repairs and Ogre2 compatibility patch, and its actual GUI has been verified on this computer.

The actual active project is `/mnt/c/Users/aston/Documents/Codex/2026-10-11/i-x20/outputs/ocean-ros-humble`. This computer has an RTX 3060 and approximately 16 GB system RAM.

## Connected drone visuals and replay recording — 24 October 2026

All drone visuals now use original Collada meshes: the orange canopy, carbon
frame, arm saddles, hardware, motor supports and bells, shafts, shaped
propellers, braced landing struts, feet, camera housing, optical glass and
navigation lights. Structural supports overlap their adjoining parts. The
rotating propeller hubs and blades sit above the motor bells.

The 15 mesh assets contain 78 closed solids and 10,992 triangles with outward
surface normals. Independent geometry review checked connectivity and sampled
1,681 camera rays to a centered 0.40 m marker at nominal minimum clearance,
without an obstruction. The generator also preserves
the complete nonvisual SDF: link poses, mass/inertia, collisions, rotor joints,
sensor poses/calibration and controller parameters are unchanged.

- [Actual close-up in Gazebo](verification/landing-visual-update/connected-drone.png).
- [Mesh review](verification/landing-visual-update/mesh-review.json) and
  [camera-clearance review](verification/landing-visual-update/clearance-review.json).
- [Model notes and regeneration instructions](ros/ocean_sim/models/landing_drone/MODEL_NOTES.md).

The updated model completed an initial camera-guided landing, then a second
landing after the actual **Replay landing** button was clicked in Gazebo.
Both runs passed all 16 independent landing criteria, including raw-image tag
detection, named foot/station contacts, motors disabled, supported height and
two simulated seconds of settled contact:

| Measurement | Initial landing | Recorded button replay |
| --- | --- | --- |
| Marker | tag36h11, ID 0 | tag36h11, ID 0 |
| Observed simulation / wall interval | 13.4100 s / 14.938 s | 26.1875 s / 33.319 s |
| Camera frames / valid tag poses received | 268 / 168 | 519 / 313 |
| Physical foot/station contact messages | 1,017 | 1,095 |
| Final center distance in station frame | 0.0453 m | 0.2742 m |
| Final drone-origin height in station frame | 0.8000 m | 0.8000 m |
| Settled support with motors disabled | 2.000 s | 2.000 s |

The distances above are computed from independent model poses for verification.
The controller uses the rendered AprilTag image for target-relative guidance.
The replay's final independent distance remains within the 0.35 m acceptance
limit; the controller's camera estimate is a separate measurement.

- [Initial landing report](verification/landing-visual-update/cycle-0/landing-report.json)
  and [recorded replay landing report](verification/landing-visual-update/cycle-1/landing-report.json).
- [Actual replay-and-landing GIF](verification/landing-visual-update/replay-landing.gif)
  and [final GUI image](verification/landing-visual-update/replay-landing.png).

The replay was triggered by an actual mouse click on **Replay landing** in the
running Gazebo window. One world-clock rewind was observed, and the same Docker
container, Gazebo server/GUI and vision-controller processes remained running
with zero container restarts. The renderer log identifies **D3D12 (NVIDIA
GeForce RTX 3060)**. The successful run has no texture-residency assertion or
process error.

The GIF contains 38.47 seconds of measured capture timing at 1200 × 1000. It
shows the landed scene, reset/arming with the replay button disabled, a fresh
flight and another supported touchdown. Capture intervals vary; the retained
frames preserve recorded timing. All frames are actual window pixels.

- [Replay summary and continuity proof](verification/landing-visual-update/replay-summary.json),
  [recording report](verification/landing-visual-update/replay-recording-report.json),
  [actual button-click record](verification/landing-visual-update/gui-click.json),
  and [capture timestamps](verification/landing-visual-update/replay-landing.json).
- [Reset/arming GUI frame](verification/landing-visual-update/replay-reset.png),
  [successful simulation log](verification/landing-visual-update/simulation.log),
  and [renderer log](verification/landing-visual-update/ogre2.log).

The first launch exited in the NVIDIA/WSLg graphics driver during initialization.
The relaunch completed both physical landings and the recorded replay. The
[first-start failure log](verification/landing-visual-update/startup-driver-failure.log)
is retained separately; the successful results do not establish that this
driver startup failure can never recur.

## Earlier replay-button verification — 24 October 2026

The measurements in this section precede the connected-mesh visual revision.
They remain the retained multi-replay regression evidence.

The landing GUI now has a **Replay landing** button in its upper-left
**Landing demo** card. It pauses physics, resets world time and all models,
clears camera/contact history and the motor-off latch, arms at paused time
zero, then resumes automatic landing. It preserves the GUI, current view,
selected tag and Docker container. The ocean visual retains GPU resources
and restores its configured wind settings when time rewinds.

Four replay actions passed in the actual RTX 3060 GUI over 80.498 wall seconds:
a five-message request burst caused one reset; a replay began after touchdown;
another interrupted that flight after three simulated seconds; and the final
replay began with physics paused. Requests used native Gazebo Transport,
exercising the same `/landing/replay` bridge as the GUI button. Three completed
flights each passed all 16 independent camera/contact/pose landing criteria:

| Completed replay | Observed simulation interval | Final center distance | Foot/station contact messages | Settled with motors off |
| --- | --- | --- | --- | --- |
| 1, initial request burst | 26.2875 s | 0.0627 m | 1,134 | 2.000 s |
| 3, interruption during flight | 18.6100 s | 0.2497 m | 953 | 2.000 s |
| 4, starting while paused | 18.5300 s | 0.2468 m | 980 | 2.010 s |

Container identity, start time, host PID and restart count remained identical.
Gazebo's launcher, server, GUI and vision-controller PIDs and start ticks also
remained identical across all four resets; there were no container restarts.
The renderer log identifies **D3D12 (NVIDIA GeForce RTX 3060)**. The run log
contains no Ogre2 texture assertions, process errors or segmentation faults.

- [Replay summary and continuity proof](verification/landing-replay/replay-summary.json),
  [complete replay report](verification/landing-replay/replay-report.json), and
  [actual final GUI screenshot](verification/landing-replay/gazebo-replay.png).
- Independent landing reports for
  [replay 1](verification/landing-replay/cycle-1/landing-report.json),
  [replay 3](verification/landing-replay/cycle-3/landing-report.json), and
  [replay 4](verification/landing-replay/cycle-4/landing-report.json), with raw
  camera frames, independent tag annotations and contact measurements.
- [Simulation log](verification/landing-replay/simulation.log) and
  [actual renderer log](verification/landing-replay/ogre2.log).

The real QML button was separately clicked in a Qt test view: one request per
click, disabled while resetting, rejection of a stale ready message and a second
successful replay click all passed. See the
[GUI click-test log](verification/landing-replay/gui-button-test.log).
The actual Gazebo window was visually inspected; its replay control and live
camera remained open throughout the runtime check.

Runtime verification found a native Harmonic contact-system reset bug: its
reconstructed system discovers sensors only through `EachNew`, after reset has
already cleared those flags. A regression test reproduced zero contact messages
after the first native reset. The supplied `ReplayContact` system scans all
sensors on initialization/reset and restores collision contact-data components.
It passed initial contact plus two resets, preserving scoped topics and actual
collision names. See the [native baseline](verification/landing-replay/contact-native-baseline.json),
[passing replacement](verification/landing-replay/contact-reset-regression.json),
and [station-system test log](verification/landing-replay/station-systems-tests.log).

Contact confirmation uses sensor timestamps, including out-of-order handling
and gap detection, so delayed ROS clock callbacks cannot shorten the contact
hold or erase a new contact observation. After motor shutdown, tag processing
and the preview are limited to 2 Hz to leave time for contact and replay
callbacks. Each replay restores normal flight processing. The raw camera
remains configured at 20 Hz, with the annotated flight preview limited to 5 Hz.

The one-time upgrade requires rebuilding and launching the updated landing
image. Subsequent replays use the button without restarting Gazebo or Docker.
Replay rewinds `/clock`; finish a ROS bag before starting another run.

## Earlier live-camera panel and single-flight recording

The measurements and images in this section precede the connected-mesh visual
revision. The current replay recording is linked above.

The landing world now opens Gazebo's native `ImageDisplay` in the upper right,
showing the actual downward image with detected tag corners and pose axes,
tag family/ID, guidance state, camera height, lateral error and motor state.
The annotated RGB preview is 640 × 480, limited to 5 Hz, with a one-way
ROS-to-Gazebo image bridge and queue depth 1. Pose estimation still uses the
original calibrated 960 × 720 camera frames, configured at 20 Hz. Preview size
and rate limit GUI traffic; they do not change the detector's calibration.

The recorded flight passed all 16 physical landing criteria while the live
panel and GUI recorder were active:

| Measurement | Recorded live-panel flight |
| --- | --- |
| Marker | tag36h11, ID 0 |
| Observed simulation / wall interval | 21.4025 s / 23.956 s |
| Camera frames / valid tag poses received | 426 / 298 |
| Physical foot/station contact messages | 833 |
| Final center distance in station frame | 0.0644 m |
| Final drone-origin height in station frame | 0.79998 m |
| Maximum station horizontal displacement during observation | 0.1126 m |
| Station vertical range during observation | 0.8396 m |
| Settled support with motors disabled | 2.005 simulated seconds |

An earlier live-panel flight also passed (0.176 m center distance, 975 contact
messages); its view was reframed for the final recording. The table and retained
report describe the fixed-view flight shown in the GIF. The GUI renderer log
again identifies **D3D12 (NVIDIA GeForce RTX 3060)** and OpenGL 4.2.

- [Continuous side-view landing GIF](verification/landing-live/side-view-landing.gif):
  54 consecutive actual framebuffer captures over 27.01 s, at requested 0.5 s
  intervals, proportionally resized to 960 × 800 with a shared GIF palette.
  No frames were generated or interpolated; playback preserves measured timing.
- [Capture timestamps](verification/landing-live/side-view-landing.json),
  [recording timeline](verification/landing-live/recording-timeline.json), and
  [final GUI frame](verification/landing-live/side-view-landing.png).
- [Passing report](verification/landing-live/landing-report.json),
  [probe log](verification/landing-live/probe.log),
  [renderer log](verification/landing-live/ogre2.log),
  [runtime identity](verification/landing-live/runtime.json), and
  [simulation log](verification/landing-live/simulation.log).

One initial startup exited inside the NVIDIA WSL D3D12 driver during Ogre
initialization. Restarting the same container recovered; two subsequent full
GUI flights passed. This driver startup failure is retained in the
[initial attempt log](verification/landing-live/startup-driver-failure.log);
these results do not establish that it can never recur. No texture-residency
assertion occurred in the successful runs.

The preview keeps its last image if camera messages stop. The controller's
timer still requests hover after target loss; `/landing/status` carries its
current state independently of the image. Gazebo's
[ImageDisplay implementation](https://github.com/gazebosim/gz-gui/blob/gz-gui8/src/plugins/image_display/ImageDisplay.cc)
requires RGB rather than BGR for this path, and the preview converts accordingly.
Static source, shell, XML/resource and Compose checks passed after the change.

The ocean-only action remains `bash scripts/run-wsl.sh start`: it loads the
original ocean world with the duck and buoy, without the station, drone or
landing camera panel. The landing panel is added only to the generated world.

## Floating station and drone landing — 24 October 2026

The updated image adds a physically buoyant moored station, a calibrated downward
camera, configurable AprilTag generation, visual landing control, and real
landing-foot/deck contact sensing. Both complete landing runs passed:

| Measurement | GUI baseline | Alternate tag, headless camera |
| --- | --- | --- |
| Marker | tag36h11, ID 0 | tag25h9, ID 7 |
| Verification interval, simulation time | 26.305 s | 26.310 s |
| Camera frames received | 524 | 523 |
| Valid tag-pose messages received | 525 | 511 |
| Physical foot/station contact messages | 903 | 702 |
| Final center distance in station frame | 0.200 m | 0.230 m |
| Final drone-origin height in station frame | 0.800 m | 0.800 m |
| Maximum station horizontal displacement during observation | 0.126 m | 0.127 m |
| Station vertical range during observation | 1.008 m | 1.008 m |
| Settled support with motors disabled | 2.0 simulated seconds | 2.0 simulated seconds |

The station is dynamic, with a 3 × 3 × 0.7 m hull and 1800 kg mass. Its mooring
holds horizontal position with a 20,000 N/m spring and 12,000 Ns/m damping;
vertical wave response and tilt remain active. The drone starts at
`(0.8, -0.5, 3.0)` m and uses a 960×720, 20 Hz downward camera. The marker's
black-border size is 0.40 m, with an additional 0.10 m white margin per edge.

The visual outer controller estimates metric relative pose from the actual
camera image and calibration. It does not subscribe to station or drone world
positions. The independent verifier uses both model poses to check centered,
supported contact. Gazebo's built-in multicopter attitude/velocity inner loop
does use simulation state and drives four motor models; this is a simulation
controller, not a PX4/ArduPilot integration.

Landing is accepted only with independently decoded raw images, tag poses,
physical named foot/hull contacts, motors disabled, all feet within the deck,
correct relative height, limited horizontal station drift and measured wave
heave. Message counts differ slightly because subscribers start independently.
The baseline's final reported image age was 22.5 ms in simulation time, and its
processing-time moving average was 29.6 ms; these are diagnostic samples, not
a general performance guarantee.

- [Baseline report](verification/landing/landing-report.json),
  [relative pose samples](verification/landing/relative-pose-samples.json), and
  [GPU renderer log](verification/landing/ogre2.log), plus
  [final image/runtime identity](verification/landing/runtime.json).
- [Alternate-family/ID report](verification/landing-alternate/landing-report.json)
  and [simulation log](verification/landing-alternate/simulation.log).
- [Actual station after touchdown](verification/landing/gazebo-station.png) and
  [post-touchdown wave animation](verification/landing/station-after-landing.gif).
- [Three recorded camera stages](verification/landing/camera-stages.gif),
  [raw final camera image](verification/landing/final-camera.png), and
  [independently annotated final image](verification/landing/final-annotated.png).

The image and camera evidence comes from the actual RTX 3060 run. The GUI was
framed through Gazebo's camera service and read through WSLg's X11 window.
The camera sequence contains three recorded stages; the GUI animation was
captured after touchdown and is not presented as a continuous flight recording.

Additional checks passed: seven camera/guidance tests including calibrated
OpenCV pose recovery; the compiled C++ mooring test for force signs, damping,
saturation and yaw wrapping; standalone validation of the generated alternate
world; and the original ocean smoke test again (8.00 s, 0.526 m buoy range).
The graphics check continues to identify the RTX 3060 through D3D12/OpenGL 4.2.

Runtime integration corrected Harmonic's contact-topic behavior: the Contact
system uses native scoped per-foot topics rather than the outer sensor topic.
Four Gazebo-to-ROS bridges now combine those topics into `/drone/contacts`.
Camera input keeps only the latest frame, OpenCV uses one thread, and optional
debug images are limited to 5 Hz and generated only for subscribers. Losing a
fresh tag pauses descent. Controller or bootstrap process failure shuts down
the scene instead of leaving a stale velocity command active.

These tests cover the supplied initial view, platform and sea state. They do
not establish global tag search, real-aircraft performance, onboard state
estimation, performance in arbitrary weather, or precision landing after
changing dimensions without recalibration. See [LANDING.md](LANDING.md).

## Base ocean runtime results — 23 October 2026

- Reproduced the reported standalone validator failure: all three `model://` includes were unresolved with only `GZ_SIM_RESOURCE_PATH`.
- The same world validates as `Valid.` when the standalone command receives the local model roots in `SDF_PATH`.
- Rebuilt the ASV C++ plugins with the compatibility patch and installed the final world layout.
- Full headless smoke test passed on the final image: simulation clock advanced **8.00 s**, and the buoy's settled vertical range was **0.526 m**, while remaining near the water surface.
- The same probe also passed inside the actual running GUI simulation: clock advanced **8.00 s**, buoy vertical range **0.557 m**.
- Actual GPU check passed: **Accelerated: yes**, **D3D12 (NVIDIA GeForce RTX 3060)**, **OpenGL 4.2**, **Mesa 23.2.1**.
- The GUI's own `/root/.gz/rendering/ogre2.log` independently identifies **D3D12 (NVIDIA GeForce RTX 3060)** and OpenGL 4.2. This confirms the renderer used by Gazebo itself.
- Inspected actual WSLg window captures: reflective blue water, wave crests, a yellow duck and a red buoy render. Successive frames show the waves and floating objects moving.
- Eight GUI frames were captured at a requested 0.5-second interval. In a fixed water-only region excluding objects, grid and controls, **43.35%** of pixels changed by more than 3 intensity levels in at least one channel between the first and last frame. This corroborates visual animation; it is not a frame-rate benchmark.
- The final GUI and server remained running for more than **4 minutes**, with zero container restarts, no OOM kill and no recurrence of the texture-residency assertion. The patched GUI also survived a deliberate container recreation during layout verification.
- An intentionally missing model correctly fails validation and reports the validation log and failure stage. Cleanup no longer attempts to tail a nonexistent simulation log.

## GUI evidence

- [Actual Gazebo window screenshot](verification/gazebo-ocean.png).
- [Actual wave-animation capture](verification/gazebo-ocean.gif), resized from 1200×1000 to 960×800 for playback.
- [GUI renderer log](verification/ogre2.log), including the GPU identification and retained startup warnings.
- [Gazebo startup log](verification/gazebo-startup.log), including `WavesVisual: skipping duplicate visual plugin for entity [6]`.
- [Image identity and runtime report](verification/run-report.json), [running GUI/server processes](verification/processes.txt), and [installed build information](verification/build-info.txt).
- [Capture measurements](verification/capture-metrics.json).

These images are captured pixels from the live Gazebo window, not generated illustrations. The capture helper read Gazebo's X11 window through WSLg because the Windows desktop inspection helper could not initialize. The simulation was left running after verification.

## Repairs made

The earlier crash was reproduced in `Ogre2DisplacementMap::InitTextures` / `TextureGpu::setResolution`: a second GUI WavesVisual instance retrieved the first instance's already-resident textures and attempted to set their resolution again. [Gazebo issue #1647](https://github.com/gazebosim/gz-sim/issues/1647) documents repeated visual-plugin initialization.

The build now applies [asv-wave-visual-dedup.patch](patches/asv-wave-visual-dedup.patch) to pinned ASV commit `ca8629df4e191235753dfae92ef725d30b923364`. A mutex-protected registry permits one WavesVisual owner for each Entity Component Manager/entity pair; duplicate instances stop before subscribing or allocating render resources. Cleanup disconnects the callback before releasing the registration. FFT waves, shaders and the `DYNAMIC_TEXTURE` rendering path remain active. The required `GzSceneManager` plugin stays installed. The GUI layout also uses compact utility plugins and matching viewport anchors so the ocean fills the window.

`gz sdf -k` is a standalone SDFormat tool. It does not initialize Gazebo Sim's model resolver. The smoke precheck now supplies `SDF_PATH` for that command only, preserving any existing entries. Gazebo itself continues to use `GZ_SIM_RESOURCE_PATH`.

The installed scene bridge also emits empty frame names on `/ocean/poses`. The original probe therefore collected clock messages but no named buoy samples. A local adaptation of the upstream spherical buoy now adds Gazebo's standard PosePublisher. Its dedicated `/ocean/buoy/pose` topic bridges to `geometry_msgs/msg/PoseStamped`; the smoke probe uses that topic. The upstream buoy's geometry, mass, inertia and hydrodynamics are preserved. `/ocean/poses` retains its previous message type for compatibility.

## Completed checks

- Python source and ROS launch syntax.
- Bash script syntax and Linux line endings.
- Dockerfile shell-command syntax.
- Compose YAML, NVIDIA GPU reservation, WSLg mounts and DirectX device mapping.
- Docker Compose's own configuration validation with the optional development overlay.
- Persistent workspace/recording mount configuration and ROS bag dependency declaration.
- ROS bridge types, topic names and smoke-test subscriber compatibility.
- XML structure for the package, world and surface model.
- Local resolution of the ocean, buoy, duck, mesh, GLSL shader and DDS texture files.
- Matching wave settings for physical forces and visual rendering.
- ROS package installation includes all world, launch, bridge and model files.
- Dockerfile source pin matches the inspected ASV Wave Sim commit.
- GPU-check parsing accepts accelerated RTX 3060 results with OpenGL 4.2/4.6 and rejects software rendering, unaccelerated rendering, wrong adapters and insufficient/missing core profile versions. These were simulated diagnostic outputs, not GPU measurements.
- Wave-command help and numeric input validation.
- Windows prerequisite checker syntax and successful read-only execution in PowerShell and Windows PowerShell 5.1.
- Updated runner help and Bash syntax, including its existing-container shell and development-overlay actions.
- Independent review of upstream Harmonic dependencies, plugin filenames and resource paths.

## Re-run the checks

From the project directory in Ubuntu:

```bash
bash scripts/run-wsl.sh check
bash scripts/run-wsl.sh smoke
```

`check` measures the container's actual OpenGL renderer. `smoke` checks the loaded world, advancing ROS simulation time and buoy motion near the sea surface. To rebuild and reopen the GUI after updating these files:

```bash
bash scripts/run-wsl.sh build
bash scripts/run-wsl.sh restart
```

## Scope and remaining warnings

The fatal texture-residency assertion is fixed in the tested configuration. The full Ogre log still contains material fallback, mapped-buffer, shader-validation and staging-texture-budget warnings. The staging-budget warnings occurred during initialization; a generated shader also logged a sampler-validation warning later in the run. These did not prevent the captured ocean animation or the GUI simulation's successful ROS/physics probe. This is not a claim of warning-free Ogre rendering or a performance benchmark. Larger scenes, additional cameras, multiple ocean surfaces and long-duration runs have not been stress-tested.
