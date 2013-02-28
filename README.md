Move Base 3D 
===================================

Daniel Perea Strom  
<riverstrom@gmail.com>

Copyright (C) 2013, Universidad de La Laguna  
Grupo de Robótica de la Universidad de La Laguna  
<http://grull.isaatc.ull.es/>
 
Overview
-----------------------------------

The stack contains a 3D path (3D translations + yaw) planner based on motion primitives. 

This code is at an experimental stage, and licensed under the GPLv3 license.

Installing
-----------------------------------

### From source ###

Create a directory where you want the package downloaded (ex. `~/ros`), 
and make sure it's added to your`$ROS_PACKAGE_PATH`.

Cd to the newly created directory (ex. `cd ~/ros`),

If you don't have git installed, do so:

    sudo apt-get install git-core

Download the stack from the repository:

    git clone https://github.com/riverstrom/move_base_3D.git
    
You will also need the interactive marker package:

    git clone https://github.com/riverstrom/goal_interactive_marker.git

Install the dependencies manually by:

    sudo apt-get install ros-fuerte-sbpl
    sudo apt-get install ros-fuerte-arm-navigation-experimental

Compile the stack:

    rosmake move_base_3D
    
And compile the interactive marker package:

    rosmake goal_interactive_marker

Quick usage
-----------------------------------

The move_base_3D.launch file will start all needed nodes:

    roslaunch move_base_3D move_base_3D.launch 

Launch RViz. For convenience, there is a launch file with a nice configuration:

    roslaunch move_base_3D rviz_interaction.launch 

In RViz, press the "Interact" button (or I in the keyboard) to highlight the Start
and Goal interactive markers. Click or move the goal marker to compute a new plan.

You can move the Start and Goal markers to wherever you want, but a new plan is computed
only when the Goal marker is changed. Check the output messages for problems with either Start or Goal poses. If a WARN "Solution not found" appears it could be that you chose a pose too close to 
the obstacles (or actually there is no solution to the problem).

There is a visualization bug (maybe in RViz) where trailing arrows markers are left behind. 
Just uncheck and check again the "Plan markers" display in RViz. The last plan will show up cleanly.


