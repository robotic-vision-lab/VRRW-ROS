# Unity Compatible Unified Robot Description Format (URDF)

These URDFs have been modified to work with Unity asset paths instead of ROS package path. For example:

```xml
<mesh filename="package://unity_robot_description/models/ur5e/visual/base.dae"/>
```

is changed to

```xml
<mesh filename="package://Meshes/ur5e/visual/base.dae"/>
```

Assuming that there is a `Meshes` folder containing the correct paths e.g., `Meshes/[ur5e, etc.]`. To set this up, just drag the `models` folder into wherever the URDF is, and rename it to `Meshes`.

Also, addition collisions were disabled due to some issues with how Unity's Temporal Gauss Seidel Solver causes some internal collisions, see Unity-Technologies/Unity-Robotics-Hub#216 and Unity-Technologies/Unity-Robotics-Hub#301. These collisions are saved in `robotiq_extra_collisions.urdf` 