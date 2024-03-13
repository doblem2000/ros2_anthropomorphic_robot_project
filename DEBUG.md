# Debug

The easiest way to debug ROS2 programs is through VSCode [ROS extension](https://github.com/ms-iot/vscode-ros).
Please give a look at the official repository documentation before reading next.

## How to build ROS2 code in VSCode

Everything you need to know to build ROS2 code in VSCode is explained in [this video tutorial](https://www.youtube.com/watch?v=k2TLdXHjVsU) (also linked in the official repo documentation).

## How to debug ROS2 nodes

In order to debug ROS2 nodes, first make sure they are built with the debug flag.
A convenient way of doing so is through colcon's mixins (installation instructions [here](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html#install-ros-2-and-colcon)):

```bash
colcon build --mixin debug
```

The VSCode ROS extension enables new "Run and Debug" configurations that can be used to easily debug ROS2 nodes.
In particular, we refer to the **ROS: Launch** configuration shown below.

![launch](./vscode_ros_launch.png)

Press "play" and just follow what suggested by the interactive procedure:

* Select ROS: Launch
* Select package from drop-down menu
* Select launch file by typing in the text box

In order not to repeat the procedure for every debug session, it might be convenient to add the following configuration to your `launch.json` file:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "ROS: Launch custom file",
            "request": "launch",
            "target": "<full path to your launch file in the installation folder>",
            "launch": ["rviz", "gz", "gzserver", "gzclient"],
            "type": "ros"
        }
      ]
}
```

With this configuration available, it suffices to select it to debug your node:

![custom_launch](vscode_custom_launch.png)

Note: as also noted in the extension documentation, nodes cannot be debugged through the mentioned process if they are not launched by a launch file.
In ROS2, you can create a simple `.py` file (if not already available) just to the purpose of debugging.

## How to debug ROS2 .py launch files

The VSCode ROS extension also supports debugging ROS2 launch files in the form of Python programs.
This can be done through the configuration shown in the screenshot below.

![launch](./vscode_python_launch.png)

As for nodes,

* Select ROS: Debug Launch File
* Select package from drop-down menu
* Select launch file by typing in the text box

**Note: the launch file selected above is the one in the package's installation folder (under the `install` folder in the colcon workspace), therefore breakpoints should be put there.
Setting breakpoints in the related launch file in the `src` folder has no effect on the debugger!**
