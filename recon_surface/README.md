# ROS Package: recon_surface
Package for Surface Reconstruction from pointclouds. This package is a ROS wrapper from the original standalone version from  https://github.com/verlab/mesh-vr-reconstruction-and-view

It allows a mesh reconstruction from a PointCloud2 and Point messages. It outputs *.OBJ and *.STL files to visualize in Rviz.

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
git clone https://github.com/ITVRoC/espeleo_planner
cd ~/catkin_ws
catkin build recon_surface
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

This package starts the service `/mesh_from_pointclouds` wich receives a:
- `sensor_msgs/PointCloud2` which is the point cloud object we want to reconstruct
- `geometry_msgs/Point` which is the start point of the reconstruction (this point generally is the current position of the robot)

The service returns the mesh's filepath and also publish mesh at the `/reconstructed_mesh_marker_normal` and `reconstructed_mesh_marker_inverted` topics.
The `reconstructed_mesh_marker_inverted` topic is the same original mesh but with the normals flipped to fix a visualization problem with RViz. In this sense is required to visualize the normal and inverted mesh files to prevent missing any part of the mesh.