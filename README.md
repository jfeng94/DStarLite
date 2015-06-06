# DStarLite
Me/CS 132 Project

Implementation of DStarLite for Me/CS132b at Caltech.



#Game plan
##Data structures-
###Struct Point: Simple representation of 2D point in Euclidean space
####Public members:
#####float x, y

###Class Map:
####Private members:
#####Cell ** map
a 2D array of cells, corresponding to the locations on the field.
######float xmin, ymin, xmax, ymax
define the dimensions of the field.
#####float res is the resolution of the map.
####Public members
#####Constructor
takes xmin, xmax, ymin, ymax, res and initializes the Cell map
#####init(Point start, Point end)
takes two positions, and 
