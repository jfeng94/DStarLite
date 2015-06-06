# DStarLite
Me/CS 132 Project

Implementation of DStarLite for Me/CS132b at Caltech.



#Game plan
##Data structures-

###Struct Point: Simple representation of 2D point in Euclidean space
####Public members:
#####float x, y
True locations on the field

###Struct Cell
####Public members:
#####Point p

#####int xidx, yidx
Index of the cell in the map

####int status
Tells us if the cell is blocked, open, or unknown

###Class Map:
####Private members:
#####Cell ** map
a 2D array of cells, corresponding to the locations on the field.

######float xmin, ymin, xmax, ymax
define the dimensions of the field.

#####float res
is the resolution of the map.

####Public members

#####Map::Map(float xmin, float xmax, float ymin, float ymax, float res)
Constructor takes xmin, xmax, ymin, ymax, res and initializes the Cell map

#####void Map::init(Point end)
takes a position, and updates the distance of the cells in the map to
reflect that

#####Point Map::getIdx(Point p)
Takes a point and returns the indices of the closest cell in the map
