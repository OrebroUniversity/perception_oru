# graph_map_localisation
A library for mapping based on graph convention and nodes containing local maps.

Depends on Eigen, ros and. The package has to be built inside oru_perception workspace





Execute the following commands to generate documentation:
1. doxygen docs.config



How to add a new map type:

1. Start by copy the directory "template and change the name to to the desired map name"
2. replace all occurrences of the word "template" to the desired name
3. add compilation units inside CMakeLists.txt
4. add instantiation of your class inside graphfactory.cpp
5. Actually implement the functionality of your map type and registration type. Parameters

