# Espeleo Path Planner

**Dependencies:**
Use the script ``install-deps.sh`` to install all the package dependencies. After installation, copy the file ``weighted.py`` to your Networkx python library folder. Use the command ``pip show networkx`` to find its folder. Then overwrite the file in ``/networkx/algorithms/shortest_paths/weighted.py``. 

**Running:**
Several files are generated in this package and the file paths were not yet generalized. There is a need to change some variables in ``main.cpp`` in order to work in all environments.

Change the username "fred" on all ocurrencies in ``main.cpp`` and change the "catkin_ws" in case of your catkin environment has a different name. Then use ``catkin build`` to build the package.
Run the package with ``rosrun planning_integrated planning_integrated_node``. Run within the path ``~/catkin_ws/devel/lib/planning_integrated``.
