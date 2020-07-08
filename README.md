# Espeleo Path Planner

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
After installing Networkx, copy the file ``weighted.py`` to your Networkx python library folder. Use the command ``pip show networkx`` to find its folder. Then overwrite the file in ``/networkx/algorithms/shortest_paths/weighted.py``. 

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
