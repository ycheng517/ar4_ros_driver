# ar3_control
This module contains sample code for motion planning with the MoveIt API. There is currently a demo for the move group interface which has been adapted for the AR3 from the tutorials. There is much more to the MoveIt API, do check out the [official MoveIt tutorials](https://ros-planning.github.io/moveit_tutorials/index.html).

## Move Group Interface Demo
This demo is a simplified version of the official tutorial and covers various ways of specifying and planning for a goal - namely pose goals, joint-space goals, Cartesian paths and planning with path constraints. This demo has modified goals that are valid for the AR3 so that you can try it out with the simulator or your own arm. This should be sufficient to get started with basic tasks. The official tutorial also covers adding objects to the environment, which I've chosen to leave out, so you are encouraged to go through that as well.

* **rviz_tools_gui**  
  You will need to add the rviz_tools_gui to step through the demo. You can do so with `Panels -> Add New Panel -> RvizVisualToolsGui`.

* **Marker Array**  
  `Add -> By display type -> MarkerArray` to view the poses and trajectories.