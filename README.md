# hexapod_simulation
Simulation for the PhantomX hexapod in gazebo through ROS. This was created to practice creating simulations of real world robots.

To start the simulation:

Clone the repository
```bash
git clone https://github.com/Aramist/hexapod_simulation.git
cd hexapod_simulation
```
Launch simulation.launch from the hexapod_gazebo package
```bash
source devel/setup.bash
roslaunch hexapod_gazebo simulation.launch
```
This will open the gazebo gui, spawn a hexapod at the origin, and start the control interface.
Messages for controlling the hexapod through the interface are located in the hexapod_msgs package.
