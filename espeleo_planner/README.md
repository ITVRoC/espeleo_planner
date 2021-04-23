# Espeleo Path Planner


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
```
**Build surface reconstruction algorithm:**

1 - Build and install the surface reconstruction dependencies:

```
  sudo apt install libboost-all-dev libpng-dev
  wget https://github.com/CGAL/cgal/archive/releases/CGAL-4.11.tar.gz
  tar -xfz CGAL-4.11.tar.gz
  cd CGAL-4.11
  mkdir build
  cd build
  cmake ..
  make -j$(nproc)
  sudo make install
```
2 - Build surface reconstruction executable:

```
  cd include/surface_recon/build
  cmake ..
```


## Running:

Use ``catkin build`` to build the package. Run the package with ``rosrun espeleo_planner espeleo_planner_node``.
