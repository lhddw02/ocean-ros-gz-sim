# Set up the ocean simulation on a new Windows device

Follow this guide from the beginning on a new **Windows x64 computer with an NVIDIA GPU**. It installs the host requirements, builds the Docker image, and verifies the ocean GUI and ROS bridge. The reference device uses an RTX 3060 and 16 GB RAM; another NVIDIA GPU must pass the same checks.

The environment is Ubuntu 22.04 in Docker, ROS 2 Humble, Gazebo Harmonic with Ogre2, and ASV Wave Sim. Windows runs Docker Desktop and WSL2; WSLg displays Gazebo as a Windows window. ROS, Gazebo, Mesa and the wave plugins are installed by the Dockerfile.

The supplied `ocean-ros-humble.zip` contains the project files and instructions. **It does not contain the built Docker image.** Build it in step 7, or use the optional image-transfer procedure below. The included screenshots, logs and [VALIDATION.md](VALIDATION.md) describe the original RTX 3060 device; repeat verification on your new device.

## 1. Prepare the new computer

- Use a supported Windows 11 x64 release and check [Docker Desktop's current Windows requirements](https://docs.docker.com/desktop/setup/install/windows-install/). This guide covers the Windows/WSLg configuration supplied in this project.
- Enable CPU virtualization in BIOS/UEFI if needed. Windows Task Manager → Performance → CPU should show **Virtualization: Enabled**.
- Start with 16 GB RAM or more for a practical development setup. Plan roughly 25 GB of free storage for Docker, the image and build cache, plus space for recordings; this is a planning allowance, not a measured image size.
- The first build needs access to Docker Hub, Ubuntu and ROS package repositories, OSRF's Gazebo packages, and GitHub. Configure your organization's proxy in Docker Desktop if required.

Install the current NVIDIA **Windows** driver for your GPU from [NVIDIA's driver page](https://www.nvidia.com/Download/index.aspx), then restart if requested. In PowerShell, run:

```powershell
nvidia-smi
```

Confirm that it lists the intended GPU. If the command is unavailable, check the driver installation and reopen PowerShell. WSL uses the Windows NVIDIA driver; do not install a Linux NVIDIA display driver in Ubuntu or in the container. See [NVIDIA's WSL guide](https://docs.nvidia.com/cuda/wsl-user-guide/).

## 2. Install WSL2 and Ubuntu 22.04

Open **PowerShell as administrator** and run:

```powershell
wsl --install -d Ubuntu-22.04
```

Restart Windows if requested. Open **Ubuntu 22.04** from Start and finish creating a Linux username and password. The password is not displayed while typing.

Back in PowerShell, run:

```powershell
wsl --update
wsl --version
wsl --list --verbose
```

The Ubuntu-22.04 entry must show `VERSION 2`. If it shows 1, run:

```powershell
wsl --set-version Ubuntu-22.04 2
```

If Ubuntu-22.04 was already installed, use it and update WSL rather than reinstalling it. If the distribution name is unavailable, check `wsl --list --online`. Microsoft's [WSL installation guide](https://learn.microsoft.com/en-us/windows/wsl/install) covers these commands.

WSLg supplies the display and graphics integration. Keep WSL current and let it set `DISPLAY` automatically; a separate Windows X server is unnecessary for this setup. See [Microsoft's WSL GUI guide](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps).

## 3. Install Docker Desktop and enable Ubuntu integration

Download **Docker Desktop for Windows, x86_64** from [Docker's installation page](https://docs.docker.com/desktop/setup/install/windows-install/) and install it using the WSL2 backend. Start Docker Desktop and finish its initial setup.

In Docker Desktop:

1. Select **Linux containers**. If the menu offers “Switch to Linux containers,” select it.
2. Under **Settings → General**, enable **Use the WSL 2 based engine**. This option may already be enabled and hidden.
3. Under **Settings → Resources → WSL Integration**, enable **Ubuntu-22.04**.
4. Apply the settings and wait for the Docker engine to be ready.
5. Close and reopen the Ubuntu terminal.

In **Ubuntu**, run:

```bash
docker version
docker compose version
docker info --format '{{.OSType}}'
```

`docker version` must show both client and server, Compose must be available, and the last command must print `linux`. Run the project commands in Ubuntu without `sudo`. Docker Desktop supplies the Docker integration; installing another Docker Engine inside Ubuntu can cause conflicts. See [Docker's WSL2 instructions](https://docs.docker.com/desktop/features/wsl/).

Docker Desktop's NVIDIA GPU support requires its WSL2 backend, current Windows NVIDIA drivers and an updated WSL kernel. This project provides the Compose GPU reservation and WSLg mounts. A separate CUDA toolkit is not needed for the ocean renderer. See [Docker's GPU requirements](https://docs.docker.com/desktop/features/gpu/).

## 4. Transfer and extract the complete project

Copy the updated `ocean-ros-humble.zip` to the new computer. Use the Linux filesystem for this new installation so the project has a consistent path independent of your Windows username.

In **Ubuntu**, run:

```bash
sudo apt update
sudo apt install -y unzip
mkdir -p ~/simulations
cd ~/simulations
explorer.exe .
```

The last command opens this Linux folder in Windows File Explorer. Copy `ocean-ros-humble.zip` into that folder. Return to Ubuntu and run:

```bash
unzip ocean-ros-humble.zip
cd ~/simulations/ocean-ros-humble
ls -a
```

The archive contains one top-level `ocean-ros-humble/` folder. Extract it once into `~/simulations`; do not create an additional nested project folder. If the target folder already exists, choose a new empty destination rather than mixing versions.

Confirm these files and folders are present:

```text
ocean-ros-humble/
  SETUP.md
  README.md
  WORKFLOW.md
  LANDING.md
  Dockerfile
  compose.wsl.yaml
  compose.dev.yaml
  compose.landing.yaml
  .dockerignore
  .gitattributes
  config/
  container/
  patches/asv-wave-visual-dedup.patch
  patches/asv-wave-reset.patch
  ros/ocean_sim/
  scripts/
  third_party/
  workspace/
  recordings/
```

Keep the entire project, including hidden files, patches, shader/texture assets and license notices. All subsequent Ubuntu commands assume this directory:

```bash
cd ~/simulations/ocean-ros-humble
```

To run the optional Windows prerequisite checker, open **PowerShell** and use your extracted Linux project's actual path, replacing `YOUR_LINUX_USER` with the Ubuntu username:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File "\\wsl.localhost\Ubuntu-22.04\home\YOUR_LINUX_USER\simulations\ocean-ros-humble\scripts\check-windows.ps1"
```

This checker reports readiness without installing software or changing settings. The Ubuntu `doctor` check in step 7 is still required.

## 5. Select the NVIDIA GPU

For an RTX 3060, the existing defaults are sufficient:

```text
GPU_EXPECT=3060
MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
```

If the new computer has another NVIDIA model, create or edit a `.env` file in `~/simulations/ocean-ros-humble` and set `GPU_EXPECT` to a substring of that GPU's name. For example, for an RTX 4070:

```dotenv
GPU_EXPECT=4070
MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
```

Compose reads these GPU settings on future invocations from the project directory. On a computer with multiple NVIDIA GPUs, use the desired adapter's full name for `MESA_D3D12_DEFAULT_ADAPTER_NAME`, such as `NVIDIA GeForce RTX 3060`. An exported shell variable takes precedence over `.env`; remove stale exports if the result differs from this file.

The checker requires an accelerated **NVIDIA D3D12** renderer and OpenGL **4.2 or newer**. Changing `GPU_EXPECT` selects the expected name for verification; the new GPU must still satisfy the graphics requirements.

`BUILD_JOBS` and `OCEAN_DEV` are handled by the runner's shell logic. Configure those with shell exports or command prefixes as shown below, rather than relying on `.env`.

## 6. Optionally set WSL memory limits

Start with WSL's current defaults. If you want explicit limits on a 16 GB computer, the project includes [config/wslconfig.16gb.example](config/wslconfig.16gb.example):

```ini
[wsl2]
memory=8GB
processors=4
swap=4GB
guiApplications=true

[experimental]
autoMemoryReclaim=gradual
```

Choose limits appropriate to the new computer. To apply them, open this file in **PowerShell**:

```powershell
notepad "$env:USERPROFILE\.wslconfig"
```

Create it if needed, or merge the settings with any existing content. Ensure the filename is `.wslconfig`, not `.wslconfig.txt`. Save WSL work and quit Docker Desktop, then run:

```powershell
wsl --shutdown
```

This stops all WSL distributions. Reopen Docker Desktop, wait for its engine, and reopen Ubuntu. These are global WSL settings; see [Microsoft's configuration reference](https://learn.microsoft.com/en-us/windows/wsl/wsl-config).

## 7. Check prerequisites and build the image

In **Ubuntu**, with Docker Desktop running:

```bash
cd ~/simulations/ocean-ros-humble
bash scripts/run-wsl.sh doctor
bash scripts/run-wsl.sh config
bash scripts/run-wsl.sh build
```

`doctor` checks the Linux Docker engine, WSLg sockets, `/dev/dxg` and WSL D3D12 libraries. Resolve any failure before continuing. `config` validates the Compose configuration. The build creates the local image:

```text
ros-humble-ocean:harmonic
```

The first build downloads packages and compiles the wave plugins, so allow several minutes or longer depending on network and CPU speed. It uses two compiler jobs by default. If the compiler is killed for insufficient memory, retry with one:

```bash
BUILD_JOBS=1 bash scripts/run-wsl.sh build
```

The Dockerfile pins ASV Wave Sim to `ca8629df4e191235753dfae92ef725d30b923364`
and applies `patches/asv-wave-visual-dedup.patch`, followed by
`patches/asv-wave-reset.patch`. Keep both: they protect initial loading and
world reset from the Ogre2 texture-residency assertion. Do not remove the
world's `GzSceneManager` plugin. [Patch notes](patches/README.md) explain the fixes.

The image installs the explicit ROS Humble/Gazebo **Harmonic** bridge, `ros-humble-ros-gzharmonic`. Preserve that combination when rebuilding; Humble's default Gazebo pairing is different. No ROS or Gazebo installation is required in the host Ubuntu distribution.

ASV's source commit is pinned, while base-image and operating-system packages can change on rebuild. Use the optional saved-image method below when you want to preserve the existing installed container packages.

## 8. Verify graphics acceleration

In **Ubuntu**, run:

```bash
bash scripts/run-wsl.sh check
```

For an RTX 3060, the output should include:

```text
Accelerated: yes
OpenGL renderer string: D3D12 (NVIDIA GeForce RTX 3060)
GPU check passed: D3D12 (NVIDIA GeForce RTX 3060); OpenGL 4.2.
```

OpenGL versions above 4.2 are also acceptable. A different NVIDIA model should match your configured `GPU_EXPECT`. The OpenGL vendor can be `Microsoft Corporation`; this is normal for the WSLg D3D12 path.

The Compose configuration passes `/dev/dxg`, WSL driver libraries and display sockets into the container. Mesa translates the OpenGL renderer to the Windows GPU. Microsoft's [WSLg container example](https://github.com/microsoft/wslg/blob/main/samples/container/Containers.md) describes this graphics arrangement. `nvidia-smi` alone checks NVIDIA visibility and does not establish that Gazebo's OpenGL renderer uses the card.

## 9. Run the world and ROS smoke test

```bash
bash scripts/run-wsl.sh smoke
```

The final result should be:

```text
PASS: world loads, ROS simulation time advances, and the floating buoy moves vertically.
```

The test validates local models, starts a headless simulation, and checks advancing ROS time and buoy motion near the water surface. It reports an approximately 8-second measurement and a buoy-height range; the height range can vary between runs. The included script fixes standalone SDFormat model lookup by supplying `SDF_PATH` during validation.

This test uses an isolated ROS domain and Gazebo partition. It checks physics and ROS communication; verify the GUI separately in the next step.

## 10. Launch Gazebo and verify the visible ocean

```bash
bash scripts/run-wsl.sh start
bash scripts/run-wsl.sh status
bash scripts/run-wsl.sh logs
```

Gazebo should open as a Windows window showing blue ocean waves, a yellow duck and a red buoy. This `start` action launches the ocean and floating objects without a drone or landing station. It also returns to this scene if the landing demo was running. Check that the surface and floating objects move while the simulation is playing. The simulation controls appear at the bottom of the window. `status` should show `ocean-sim` running.

Press `Ctrl+C` to stop following logs; Gazebo continues in the background. In Ubuntu, read the **GUI's own** renderer identification:

```bash
docker exec ocean-sim sh -c "grep -E 'GL_VERSION =|GL_RENDERER =' /root/.gz/rendering/ogre2.log"
```

Confirm that `GL_RENDERER` identifies the intended NVIDIA GPU through D3D12. If the log is not present immediately, wait until the window has initialized and retry. The reference image may emit nonfatal material or shader warnings; the acceptance criteria are correct visible animation, the intended GPU, successful probes and no texture assertion or crash.

You can also check ROS time and buoy motion in this actual GUI simulation:

```bash
docker exec ocean-sim /opt/ocean/container/entrypoint.sh python3 /opt/ocean/container/smoke_probe.py
```

Leave the simulation playing until this probe completes. For visual comparison, [verification/gazebo-ocean.gif](verification/gazebo-ocean.gif) contains the original device's actual GUI capture.

| Acceptance check | Required result on the new device |
| --- | --- |
| `doctor` and `config` | Prerequisites and Compose configuration pass |
| `build`, or loading a saved image | Local `ros-humble-ocean:harmonic` image exists |
| `check` | Accelerated NVIDIA D3D12 renderer; OpenGL 4.2 or newer |
| `smoke` | World, ROS clock and buoy motion pass |
| GUI and its Ogre2 log | Animated ocean and floating objects on the intended GPU; no texture crash |
| Probe inside the GUI simulation | Clock advances and buoy moves near the surface |

## 11. Open a ROS shell and use the simulation

Open a second **Ubuntu** terminal:

```bash
cd ~/simulations/ocean-ros-humble
bash scripts/run-wsl.sh exec
ros2 topic list
ros2 topic echo /clock --once
ros2 topic echo /ocean/buoy/pose --once
set-waves --help
```

`exec` opens a sourced ROS shell inside the running container. `/ocean/buoy/pose` contains the buoy's `geometry_msgs/msg/PoseStamped`. The raw `/ocean/poses` stream has empty entity frame names in the tested bridge configuration, so use the dedicated buoy topic for measurements. Set `use_sim_time:=true` in your own ROS nodes. Type `exit` to return to Ubuntu.

The host project's `workspace/` is mounted at `/workspace`, and `recordings/` at `/recordings`. Keep ROS development files and recorded data there so they survive container removal. [WORKFLOW.md](WORKFLOW.md) covers controller development, changing wave settings, bag recording and the optional development overlay; use `~/simulations/ocean-ros-humble` as your project path when following it on the new device.

## 12. Stop, resume and apply changes

From the Ubuntu project directory:

```bash
bash scripts/run-wsl.sh stop
```

This removes the simulation container and retains the image, project, workspace and recordings. Next time, start Docker Desktop, open Ubuntu and run:

```bash
cd ~/simulations/ocean-ros-humble
bash scripts/run-wsl.sh start
```

Both `start` and `restart` recreate the simulation and reset the world. After changing image dependencies, compiled plugins, container scripts or standard image-based world files, run:

```bash
bash scripts/run-wsl.sh build
bash scripts/run-wsl.sh restart
```

`docker restart ocean-sim` restarts the old container and does not adopt a newly built image. For editable world/model/launch files, follow the `OCEAN_DEV=1` workflow in [WORKFLOW.md](WORKFLOW.md); compiled plugin changes still require rebuilding.

## 13. Run the floating-station drone landing demo

The updated image also contains a moored floating station, a configurable
AprilTag and a small drone with a downward camera. After completing the GPU and
ocean checks, run in Ubuntu:

```bash
cd ~/simulations/ocean-ros-humble
bash scripts/run-wsl.sh landing
```

Gazebo automatically opens a **Downward camera - AprilTag** panel alongside the
3D scene. The panel shows the live annotated camera image: tag outline and pose
axes, family and ID, landing state, camera height, horizontal offset and motor
state. The drone starts near the station, locates the tag in actual rendered
images, and descends onto the deck. Physical foot/deck contact must be confirmed
before its motors stop. The panel displays 640×480 images at up to 5 Hz; tag
detection uses the original 960×720 stream, configured at 20 Hz.

Click **Replay landing** in the upper-left **Landing demo** card to repeat the
flight without restarting the Docker container. It resets simulation time to
zero and restores all models to their configured spawn poses, keeps the selected
tag and current GUI view, and starts automatic landing. It also works while
paused or during flight. Wait for reset and arming to finish while the button
is temporarily disabled.

To change the marker on a fresh flight:

```bash
TAG_ID=7 bash scripts/run-wsl.sh landing
```

The isolated end-to-end check records camera evidence and a JSON report under
your persistent `recordings/` directory:

```bash
bash scripts/run-wsl.sh landing-test
```

Use [LANDING.md](LANDING.md) for the side-view landing GIF, camera topics,
families, dimensions, hover mode and tuning. The marker texture and detector
configuration are regenerated together at launch. If upgrading an older image,
run `bash scripts/run-wsl.sh build` once before `landing` to include the live
camera panel, replay button and reset support. Use the button for later flights
with the same tag and scene configuration.

To switch back to ocean waves, duck and buoy without the drone or station:

```bash
bash scripts/run-wsl.sh start
```

Both scene commands recreate the container and reset the world. After rebuilding,
use `landing` to restart the landing demo; `start` and `restart` select the ocean
and floating objects alone.

## Optional: transfer the existing built image

Use this method to avoid recompiling or to preserve the existing installed container packages. You still need steps 1–6 on the new computer, the complete project folder, and the checks in steps 8–11.

On the **original computer**, in an Ubuntu terminal with Docker Desktop running:

```bash
docker image save --output ~/ocean-ros-humble-harmonic.tar ros-humble-ocean:harmonic
```

Copy that TAR file to `~/simulations/` on the **new computer**, alongside the project ZIP, using the File Explorer method in step 4. Allow space for a large image archive. On the new computer, run:

```bash
cd ~/simulations/ocean-ros-humble
bash scripts/run-wsl.sh doctor
bash scripts/run-wsl.sh config
docker image load --input ~/simulations/ocean-ros-humble-harmonic.tar
docker image inspect ros-humble-ocean:harmonic
```

Skip the build command in step 7 and continue with `check`, `smoke` and `start`. Loading the image preserves its installed contents and tag, but Windows drivers and WSL graphics still depend on the new host and must be reverified. See Docker's [image save](https://docs.docker.com/reference/cli/docker/image/save/) and [image load](https://docs.docker.com/reference/cli/docker/image/load/) documentation.

The image archive does not include bind-mounted `workspace/` or `recordings/` data. Transfer those directories separately if you need existing work or experiments. Preserve the source project and third-party license notices with the image.

## Troubleshooting during a fresh setup

| Symptom | Check or repair |
| --- | --- |
| WSL installation reports virtualization unavailable | Enable hardware virtualization in BIOS/UEFI, restart Windows and retry WSL setup. |
| `docker` is missing in Ubuntu | Start Docker Desktop, enable Ubuntu-22.04 WSL Integration, apply changes and reopen Ubuntu. |
| Docker client works but the server is unreachable | Wait for Docker Desktop's engine and confirm Linux containers and the WSL2 backend. |
| `/dev/dxg` or WSLg sockets are missing | Confirm Ubuntu is WSL2, update WSL and the Windows NVIDIA driver, then restart WSL after saving work. |
| Display cannot open | Use Ubuntu's WSL terminal and check `echo "$DISPLAY"` and `ls /mnt/wslg`. Let WSLg supply the display variables. |
| Wrong NVIDIA GPU selected, or expected `3060` on another model | Update the project `.env` values in step 5. Use the full adapter name if needed, then recreate the simulation. |
| Renderer is `llvmpipe`/`softpipe`, or OpenGL is below 4.2 | Update Windows NVIDIA drivers and WSL; check the Compose mounts and rebuild with current Mesa packages. Do not bypass the GPU check by faking the reported OpenGL version. |
| Build is killed for insufficient memory | Retry with `BUILD_JOBS=1`; review the new computer's available RAM and optional WSL limits. |
| Missing `model://` URI or an empty SDFormat resolver callback | Use the complete updated bundle and its smoke script, then rebuild. Keep `ros/`, the ASV model resources and the sourced entrypoint. |
| Ogre2 `TextureGpu::setResolution` / residency assertion | Confirm the build contains both patches in `patches/`, rebuild, then launch `start` or `landing`. Keep `GzSceneManager`. |
| Replay button missing in the landing scene | Rebuild the updated image and run `landing` once. Use the GUI button for later replays. |
| Waves do not move | Ensure the GUI is playing, `/clock` advances, and the wave plugin loaded. Inspect the container logs. |
| The new guide's path differs from README/WORKFLOW examples | Use your new `~/simulations/ocean-ros-humble` directory consistently; paths containing the original device's username describe that device only. |

For a WSL restart, save WSL work and quit Docker Desktop before `wsl --shutdown`; then reopen both. Restarting WSL stops all its distributions and running containers.
