# Exercise1 
> Create a ROS Network

In this directory there are two folders. The folder named ``exc1`` is extraced from the `src` folder from a catkin workspace on an ubuntu 18.04 machine running ROS. It is a ROS package containing two subpackages: `publisher01` and `subscriber01` which are nodes in the ROS network.

The other folder ``plotjuggler_plot`` contains data saved from the plotjuggler application when plotting the topic `/Kastberg` and `/kthfs/result` when running the ROS network with the nodes `publisher01` and `subscriber01` active.


The `exc1` package requires `roslibpy` when executed. This can be installed via pip `pip install rospylib`.