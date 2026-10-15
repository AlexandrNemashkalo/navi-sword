# Mannequiny

Model, skin, skeleton and animations: **GDQuest and contributors**, copyright 2020.
License: [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).
See also `mannequiny.LICENSE.md` supplied with the asset.

Source: https://github.com/godotengine/godot-demo-projects/tree/master/3d/ragdoll_physics/characters
Original project: https://github.com/gdquest-demos/godot-3d-mannequin
Downloaded 2026-10-15. `mannequiny.glb` is unmodified.

NaviSword uses the authored `fight_idle_root_001` pose and adds runtime bone
rotations and a sword in `SkeletonCombatLab.cs`. These modifications are ours;
they are not an original fencing animation by GDQuest.

The GLB does **not** contain anatomical joint constraints. The demo's ragdoll
constraints are not imported: its uniform cone/twist limits are unsuitable for
an actively controlled sword arm. Our experimental constraints are applied to
the skeleton's local rotations, not physics joints:

- elbow: one bending axis derived from the authored combat pose, 8–135 degrees;
- forearm pronation/supination: ±1.3 radians along the bone;
- shoulder: bounded swing relative to the authored guard;
- wrist: elliptical envelope, ±30° deviation and ±46° flexion/extension;
- inherited bone positions and scales remain unchanged.

These are gameplay bounds, not a medically validated anatomical simulation.
The new scene isolates skeletal input; collision, locomotion, stamina and sword
inertia from the primitive prototype have not been ported to this rig yet.
