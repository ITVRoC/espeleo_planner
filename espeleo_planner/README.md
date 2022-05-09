# Espeleo Path Planner

### Last version of the point cloud planner explorator

Terminal 1:
`roscore`

`coppelia`

Coppelia maps:
  - stage_cave_v2_robot2_obstacle_T

`roslaunch espeleo_vrep_simulation espeleo_sim.launch`

`rviz -d /home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/rviz/espeleo_coppelia_white_PCLOUD.rviz`

Terminal 2:

roslaunch espeleo_planner experiment_evaluation_pcloud_rrt.launch

roslaunch espeleo_planner publish_frontier_services.launch

roslaunch espeleo_lego_loam espeleo_lego_loam_sim.launch

rosrun espeleo_planner mesh_planner_exploration_node_RRT.py

rosrun espeleo_planner exploration_metrics.py

### Launch an exploration instance

Run in different terminals:

`roscore`

`coppelia`

`roslaunch espeleo_vrep_simulation espeleo_sim.launch`

`roslaunch espeleo_planner experiment_evaluation.launch`

`roslaunch espeleo_planner exploration_services.launch`

<!---
### to launch the experiments

roslaunch espeleo_planner exploration_services.launch

roslaunch espeleo_planner experiment_evaluation.launch

### normal launch files

roslaunch espeleo_vrep_simulation espeleo_sim.launch

roslaunch espeleo_control vector_field.launch

roslaunch espeleo_lego_loam espeleo_lego_loam_sim.launch

roslaunch recon_surface recon_service.launch

roslaunch espeleo_planner espeleo_mapping_lidar.launch

roslaunch espeleo_planner mesh_planner.launch

roslaunch espeleo_teleop keyboard.launch

coppelia
-->

# convert stl to pointcloud for testing
snap run cloudcompare.CloudCompare

# convert stl to octomap
https://www.patrickmin.com/binvox/
https://www.patrickmin.com/binvox/binvox.html
https://github.com/beezees/ambientocclusion/tree/master/Source/binvox-0.4/vox/binvox
https://www.patrickmin.com/viewvox/
https://github.com/OctoMap/octomap/blob/devel/octomap/src/binvox2bt.cpp

mkdir /tmp/mesh/
./binvox -d 256 -e /tmp/mesh/mesh.stl && rosrun espeleo_planner binvox2bt /tmp/mesh/mesh.binvox
octovis /tmp/mesh/mesh.binvox.bt

## Dependencies:

**Pymesh**

This is the library used for Mesh manipulation in Python. Future works will remove this dependency and use other more friendly library.

1 - Install PyMesh dependencies
```
  $ sudo apt install libeigen3-dev libgmp-dev libgmpxx4ldbl libmpfr-dev libboost-dev libboost-thread-dev libtbb-dev python3-dev
  $ sudo python3 -m pip install numpy
  $ sudo python3 -m pip install scipy
```

2 - Clone PyMesh repository:
```
  $ git clone https://github.com/PyMesh/PyMesh.git 
  $ cd PyMesh
  $ git submodule update --init
  $ export PYMESH_PATH=`pwd`
```

3 - Install CMake 3.11.0 or newer. The easiest and less invasive way to do this is through Snap. Install Snap and CMake via snap (This command will install the last version of CMake under Snap):
  ```
    $ sudo apt update
    $ sudo apt install snapd
    $ sudo snap install cmake --classic
  ```

4 - Edit the build file to use snap CMake instead of system CMake. Open the file ``setup.py`` with your prefered text editor. Look for these lines to edit:

```
   commands = [
                "cmake .. -DCMAKE_BUILD_TYPE=Release" + cmake_args,
                "cmake --build . --config Release -- -j {}".format(num_cores),
            ] + (["cmake --build . --target install"] if want_install else [])
```

And replace by this:

```
   commands = [
                "sudo snap run cmake .. -DCMAKE_BUILD_TYPE=Release" + cmake_args,
                "sudo snap run cmake --build . --config Release -- -j {}".format(num_cores),
            ] + (["cmake --build . --target install"] if want_install else [])
```

5 - Install PyMesh:
```
   $ python ./setup.py build
   $ sudo python ./setup.py install
```

**Install other Python Dependencies:**
```
  $ pip install vtk
  $ pip install networkx
  $ pip install prettytable
  $ pip install tqdm
  $ pip install pybullet
  $ pip install python-pcl
  $ pip install sys
  $ pip install traceback
  $ pip install multiprocessing
  $ pip install numpy
  $ pip install scipy
  $ pip install sklearn
  $ pip install heapq
  $ pip install itertools
  $ pip install prettytable
  $ pip install enum
  $ pip install matplotlib
  $ pip install mayavi
  $ pip install timeit
  $ pip install pyquaternion
  $ pip install pybullet
  $ pip install graphviz
```

**Build surface reconstruction algorithm:** 

For building the `recon_surface` package, refer to: [Recon Surface README](https://github.com/ITVRoC/espeleo_planner/tree/master/recon_surface)

## Running:

Use ``catkin build espeleo_planner`` to build the package. 

Run the package with:

``roslaunch recon_surface recon_service.launch``
``rosrun espeleo_planner mesh_planner_manual.launch``

This node in the manual version subscribes to the following topics:

- `/laser_cloud_surround2` (PointCloud2) pointcloud of the environment
- `/integrated_to_init2` (Odometry) estimate robot position
- `/clicked_point` (PointStamped) to determine the goal location. This point cloud be easily sent using RViz point option in the GUI while clicking in a pointcloud point or mesh face