# acg_resources_fanuc_m20ia_35m_description

The URDF provided in this package has been manually created from the kinematic informations provided in the official Fanuc M20ia/35M documentation. The dynamic parameters were inferred from the provided meshes by assuming the materials the robot was built with iron.

## Usage

In order to graphically visualize (some of) the data contained in the URDF, a launch file is provided. After building, use the launch file provided with this package:

```bash
ros2 launch acg_resources_fanuc_m20ia_35m_description display.launch.py
```

RViz will be launched and the Fanuc robot visualized together with its reference frames.
Joint State Publisher's GUI can be used to move single joints.
