Ocean wave plugins and sample floating models are supplied by
[ASV Wave Sim](https://github.com/srmainwaring/asv_wave_sim), pinned to commit
`ca8629df4e191235753dfae92ef725d30b923364` in the Dockerfile.

This project modifies the pinned wave-plugin source with two local patches:
[asv-wave-visual-dedup.patch](../../patches/asv-wave-visual-dedup.patch) prevents
duplicate visual initialization, and
[asv-wave-reset.patch](../../patches/asv-wave-reset.patch) implements Gazebo's
reset interface while preserving the wave visual's GPU resources. The patches
and modified upstream source remain available in the image; the upstream
GPL-3.0-or-later license continues to apply.

The local `ocean_station_systems` package also adapts Gazebo Sim's
[Apache-2.0 Contact system](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/contact/Contact.cc)
to restore contact sensing after world reset. Its source retains the upstream
copyright notice, with the Apache license in that package's `LICENSE` file.

`models/ocean_surface/model.sdf` is adapted from its
`gz-waves-models/world_models/ocean_waves/model.sdf`. The GLSL shaders and DDS
textures in that model's `materials` directory are copied from the same source.
`models/spherical_buoy/model.sdf` also adapts the upstream spherical buoy,
retaining its geometry, inertia and hydrodynamics while adding pose telemetry.
The upstream project is licensed under GPL-3.0-or-later; `LICENSE` contains the
GPL license. Shader files also retain their upstream attribution and Apache-2.0
notice for the original UUV Simulator shader work.

The ocean mesh and duck are resolved from upstream model resource directories
inside the image. The spherical buoy resolves to the local adaptation above.
The upstream duck directory includes its own license. No online Gazebo Fuel
download is needed at runtime.
