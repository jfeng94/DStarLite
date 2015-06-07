#include <vector>
#include <queue>
#include <iostream>
#include <math.h>


#include "map.h"

// Helper struct dealing with indices and depths
struct idxDepth
{
    int x, y, depth;

    idxDepth(int X, int Y, int d)
    {
        x = X;
        y = Y;
        depth = d;
    }

    idxDepth operator=(idxDepth a)
    {
        x = a.x;
        y = a.y;
        depth = a.depth;
        return a;
    }
};

///////////////////////////////////////////////////////////////////////////////
// CELL FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

// Cell constructor that takes a point's position and its index in the map
Cell::Cell(float x, float y, int i, int j) 
{
    p.x   = x;
    p.y   = y;
    xidx  = i;
    yidx  = j;
    dist  = INT_MAX;
    state = UNKNOWN;
}

// Cell constructor that takes a point and its index in the map
Cell::Cell(Point a, int i, int j)
{
    p     = a;
    xidx  = i;
    yidx  = j;
    dist  = INT_MAX;
    state = UNKNOWN;
}


///////////////////////////////////////////////////////////////////////////////
// MAP FUNCTIONS
///////////////////////////////////////////////////////////////////////////////
// Map constructor
Map::Map(float Xmin, float Ymin, float Xmax, float Ymax, float Res)
{
    xmin = Xmin;
    ymin = Ymin;
    xmax = Xmax;
    ymax = Ymax;
    res  = Res;
    float Dx = xmax - xmin;
    float Dy = ymax - ymin;
    Nx = Dx / res + 1;
    Ny = Dy / res + 1;
    float x, y;

    for (int j = 0; j < Ny; j++)
    {
        for (int i = 0; i < Nx; i++)
        {
            x = xmin + i * res;
            y = ymin + j * res;
            Cell c(x, y, i, j);

            map.push_back(c);
        }
    }
}


// Get occupancy grid indices closest to given point
Point Map::getIndex(Point p)
{
    int i = (int) round((p.x - xmin) / res);
    int j = (int) round((p.y - ymin) / res);

    //std::cout << "Index: " << i << " " << j << "\n";
    //std::cout << map[j * Nx + i].p.x << " " << map[j * Nx + i].p.y << "\n";
    //setState(i, j, Cell::BLOCKED);
    return Point(i, j);
}

// Set up distance mapping with the given point as the goal
void Map::init(Point p)
{
    Point idx = getIndex(p);
    int i = idx.x;
    int j = idx.y;
    
    int depth = 0;
    setDist(i, j, depth);
    depth++;

    std::queue<idxDepth> queue;

    queue.push(idxDepth(i - 1, j - 1, depth));
    queue.push(idxDepth(i - 1, j    , depth));
    queue.push(idxDepth(i - 1, j + 1, depth));
    queue.push(idxDepth(i    , j - 1, depth));
    queue.push(idxDepth(i    , j + 1, depth));
    queue.push(idxDepth(i + 1, j - 1, depth));
    queue.push(idxDepth(i + 1, j    , depth));
    queue.push(idxDepth(i + 1, j + 1, depth));

    // While there are indices enqueued
    while (queue.size() > 0)
    {
        // Get the next index to consider 
        idxDepth temp = queue.front();
        
        i     = temp.x;
        j     = temp.y;
        depth = temp.depth; 

        // Check for index validity
        if (i >= 0 && i < Nx &&
            j >= 0 && j < Ny &&
            depth < getDist(i, j))
        {
            // Set distances
            setDist(i, j, depth);

            // Enqueue neighboring cells
            queue.push(idxDepth(i - 1, j - 1, depth + 1));
            queue.push(idxDepth(i - 1, j    , depth + 1));
            queue.push(idxDepth(i - 1, j + 1, depth + 1));
            queue.push(idxDepth(i    , j - 1, depth + 1));
            queue.push(idxDepth(i    , j + 1, depth + 1));
            queue.push(idxDepth(i + 1, j - 1, depth + 1));
            queue.push(idxDepth(i + 1, j    , depth + 1));
            queue.push(idxDepth(i + 1, j + 1, depth + 1));
        }

        // Remove and move on
        queue.pop();
    }
}

void Map::setBlocked(Point src, Point dst)
{


}


void Map::setOpen(Point src, Point dst)
{

}

// Mutator function that allows user to set state at a cell
void Map::setState(int i, int j, Cell::STATE s) 
{
    if (i >= 0 && i < Nx &&
        j >= 0 && j < Nx)
        map[Nx * j + i].state = s;
}

// Mutator function that allows user to set distance at a cell
void Map::setDist(int i, int j, int d)         
{
    if (i >= 0 && i < Nx &&
        j >= 0 && j < Nx)
        map[Nx * j + i].dist  = d;
}

// Print out an image of the map we have
std::ostream& operator<<(std::ostream& out, Map m)
{
    std::cout << m.map.size();
    out << "P3\n" << m.getNx() << " " << m.getNy() << "\n255\n";
    for (int i = 0; i < m.map.size(); i++)
    {
        if (m.map[i].state == Cell::OPEN)
        {
            out << "255 255 255\n";
        }
        else if (m.map[i].state == Cell::BLOCKED)
        {
            out << "0 0 0\n";
        }
        else
        {
            //std::cout << m.getDist(i / m.getNx(), i % m.getNx()) << "\n";
            out << "200 200 200" << "\n";
        }
    }
    return out;
}
