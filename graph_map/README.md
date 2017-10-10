# Graph_map Readme 
A library of middle-end mapping tools for rapid development and integration of SLAM with ROS or with other systems.
The library integrates State of the art back-end sparse graph optimization with G2O or iSAM.
GraphMap features intuitive abstract data types for registration and mapping increase reusabillity of code.


Current implementation features:
Representation: 2d/3d NDT with D2D-NDT for scan-to-map registration.

Features:
*offline(ROS) bag reader for running and evaluate SLAM on existing datasets.
*online(ROS) for deploying SLAM to target robot.
*Saving and loading of maps(easily to integrate this functionallity for any map representation)
*Options and parameter management comes naturally in Graph_map 




Execute the following commands to generate documentation:
1. doxygen docs.config



How to add a new map type:

1. Start by copy the directory "template and change the name to to the desired map name"
2. replace all occurrences of the word "template" to the desired name
3. add compilation units inside CMakeLists.txt
4. add instantiation of your class inside graphfactory.cpp
5. Actually implement the functionality of your map type and registration type parameters


