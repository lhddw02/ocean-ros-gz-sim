# ASV Wave Sim local compatibility patch

`asv-wave-visual-dedup.patch` applies to upstream commit
`ca8629df4e191235753dfae92ef725d30b923364` using `git apply` from the repository
root. It prevents duplicate `WavesVisual` instances from claiming the same
visual in the same Entity Component Manager.

[Gazebo issue #1647](https://github.com/gazebosim/gz-sim/issues/1647) describes
duplicate visual plugin entries reaching the GUI. A second instance otherwise
retrieves resident Ogre textures and attempts to set their resolution again,
which triggers an Ogre assertion. The mutex-protected registry permits one
owner for each ECM pointer and entity pair; only that owner releases its claim
after disconnecting its callback and removing its visuals.

FFT generation, `DYNAMIC_TEXTURE`, shaders, GPU texture allocation, and staging
uploads are unchanged. Keep the world's `GzSceneManager` plugin: it supplies
the GUI's scene updates.
