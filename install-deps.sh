echo "Installing dependencies from espeleo_planner ROS package"
mkdir pckg_deps
cd pckg_deps
git clone https://github.com/PyMesh/PyMesh.git
cd PyMesh
git submodule update --init
export PYMESH_PATH=`pwd`

sudo apt install libeigen3-dev libgmp-dev libgmpxx4ldbl libmpfr-dev libboost-dev libboost-thread-dev libtbb-dev python3-dev
sudo python3 -m pip install numpy
sudo python3 -m pip install scipy

echo "Installing Pymesh..."
python3 ./setup.py build
sudo python3 ./setup.py install

cd ..

echo "Installing Python Dependencies..."
sudo python3 -m pip install vtk
sudo python3 -m pip install networkx
sudo python3 -m pip install prettytable
sudo python3 -m pip install tqdm
sudo python3 -m pip install pybullet

echo "Installing CGAL-4.11..."
sudo apt install libboost-all-dev libpng-dev
wget https://github.com/CGAL/cgal/archive/releases/CGAL-4.11.tar.gz
tar -xfz CGAL-4.11.tar.gz
cd CGAL-4.11
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install

echo "Building surface reconstruction algorithm..."
cd include/surface_recon/build
cmake ..

echo "Dependencies installed! Now copy the script weighted.py to your netowrkx python library directory.
Use the command (pip show networkx) to find where networkx is installed. 
Then overwrite the file in /networkx/algorithms/shortest_paths/weighted.py"


