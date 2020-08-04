# ROS Package: recon_surface
Package for Surface Reconstruction from pointclouds. This package is a ROS wrapper from the original standalone version from  https://github.com/verlab/mesh-vr-reconstruction-and-view

It allows a mesh reconstruction from a PointCloud2 message. It outputs *.OBJ and *.STL files to visualize in Rviz.

### Installation guide

First, installing all dependencies in a terminal:

```bash
sudo apt-get install g++ cmake libboost-all-dev libeigen3-dev libgmp-dev libgmpxx4ldbl libmpfr-dev libpng-dev
```

Then, download [CGAL Library (Version 4.11)!](https://github.com/CGAL/cgal/archive/releases/CGAL-4.11.tar.gz) 

Compile the cgal library by going into the unzipped folder and executing the following in a terminal:

```bash
mkdir build ; cd build ; cmake .. ; make ; sudo make install
```

Clone this repo and build

```bash
cd ~/catkin_ws/src 
git clone XXX
cd ~/catkin_ws
catkin build
```

### Executing

Launch the service:

```bash
roslaunch recon_surface recon_service.launch
```

For testing with some sample cave pointcloud files:

```bash
roslaunch recon_surface test_recon_service.launch
```

### Services

This package starts the service '/mesh_from_pointclouds' wich receives a PointCloud2 message and outputs a filepath
 with the mesh and also publish the last mesh at the '/reconstructed_mesh_marker' topic.

