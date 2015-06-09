#include <vector>
#include <queue>
#include <iostream>
#include <math.h>
#include <cmath>
#include <limits.h>
#include <float.h>
#include <list>
#include <cstdlib>

#include "map.h"

// Helper struct dealing with indices and depths
struct idxDepth
{
    int x, y, depth;
    float f;
    idxDepth *parent;

    idxDepth()
    {
        x = 0;
        y = 0;
        f = FLT_MAX;
        depth = INT_MAX;
        parent = NULL;
    }
    idxDepth(int X, int Y, int d)
    {
        x = X;
        y = Y;
        f = FLT_MAX;
        depth = d;
        parent = NULL;
    }
    idxDepth(Point p, int d)
    {
        x = p.x;
        y = p.y;
        f = FLT_MAX;
        depth = d;
        parent = NULL;
    }

    idxDepth operator=(idxDepth a)
    {
        x = a.x;
        y = a.y;
        depth = a.depth;
        parent = a.parent;
        f = a.f;
        return a;
    }

    bool operator<(idxDepth a)
    {
        if (f < a.f)
            return true;
        else
            return false;
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

    return Point(i, j);
}


Point Map::OccupancyToReal(Point p)
{
    int i = p.x;
    int j = p.y;
    float x = map[Nx * j + i].p.x;
    float y = map[Nx * j + i].p.y;
    return Point(x, y);
}

// Set up distance mapping with the given point as the goal
void Map::init(Point p)
{
    Point idx = getIndex(p);
    int i = idx.x;
    int j = idx.y;

    // Set data members for goal;
    goal = p;
    goal_idx = Point(i, j);
    path.push_back(goal);
    int depth = 0;
    setDist(i, j, depth);
    depth++;

    std::queue<idxDepth> queue;

    // TODO: Refactor so it's not so fucking ugly
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

            if (maxDist < depth)
                maxDist = depth;
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

void Map::recalculate()
{
    // Reset all distances to -1, aside from blocked cells
    for (int n = 0; n < Ny; ++n)
    {
        for (int m = 0; m < Nx; ++m)
        {
            if (getDist(m,n) != INT_MAX)
            {
                setDist(m, n, -1);
            }
        }
    }

    // Set goal to depth 0
    int i = goal_idx.x;
    int j = goal_idx.y;
    int depth = 0;

    // Place goal into queue.
    std::queue<idxDepth> queue;
    queue.push(idxDepth(i, j, depth));

    // While there are changes pending
    while (queue.size() > 0)
    {
        // Get the next index to consider 
        idxDepth temp = queue.front();
        
        i     = temp.x;
        j     = temp.y;
        depth = temp.depth; 

        //std::cout << "Checking " << i << " " << j << " with depth " << depth << "\n";
        // Check if this position is eligible for update
        if (i >= 0 && i < Nx && j >= 0 && j < Ny &&
            (depth < getDist(i, j) || getDist(i, j) == -1) &&
            getDist(i, j) != INT_MAX)
        {
            //std::cout << "Is valid! Setting dist.\n";
            setDist(i, j, depth);

            // Update maxDist
            if (maxDist < depth)
                maxDist = depth;
                
            //std::cout << "Check neighboring cells...\n";
            // Enqueue neighboring cells
            for (int n = j - 1; n < j + 2; ++n)
            {
                for (int m = i - 1; m < i + 2; ++m)
                {
                    //std::cout << "Cell: " << m << " " << n << "\n";
                    // Check for valid indices && not blocked
                    if(m >= 0 && m < Nx && n >= 0 && n < Ny)
                    {
                        //std::cout << "Looks okay. Pushing in with depth " << depth + 1 << "\n";
                        queue.push(idxDepth(m, n, depth + 1));
                    }
                }
            }
        }

        // Pop off and move on
        queue.pop();
    }
}

// Function that returns the minimum depth of neighbors surrounding a 
// given cell
int Map::checkNeighbors(Point idx)
{
    int i = idx.x;
    int j = idx.y;

    int min = INT_MAX;

    // Get min
    for (int m = j - 1; m < j + 2; m++)
    {
        for (int n = i - 1; n < i + 2; n++)
        {
            if (n >= 0 && n < Nx && m >= 0 && m < Ny &&
                (n != i || m != j))
                min = ((min < getDist(n, m)) ? min : getDist(n, m));
        }
    }

    return min;
}


// Function that returns the minimum depth of neighbors surrounding a 
// given cell
Point Map::neighborIndex(Point idx)
{
    int i = idx.x;
    int j = idx.y;
    int x, y;
    int min = INT_MAX;

    // Get min
    for (int m = j - 1; m < j + 2; m++)
    {
        for (int n = i - 1; n < i + 2; n++)
        {
            std::cout << getDist(n, m) << " ";
            if (n >= 0 && n < Nx && m >= 0 && m < Ny &&
               (n != i || m != j) && getDist(n, m) < min){
                min = getDist(n, m);
		            x = n;
		            y = m;
            }
        }
        std::cout << "\n";
    }
    std::cout << "\n";

    return Point(x, y);
}



// Helper function that enqueues neighbors of a given occupancy grid index
void Map::enqueueNeighbors(std::queue<Point> &q, int i, int j)
{
    for (int m = j - 1; m < j + 2; m++)
    {
        for (int n = i - 1; n < i + 2; n++)
        {
            if (n >= 0 && n < Nx && m >= 0 && m < Ny &&
                (n != i || m != j))
                q.push(Point(n,m));
        }
    }
}

// Call this function when blocking a cells out. It will update all the
// cells dependent on this one. blocked is in occupancy grid space.
void Map::updateDistances(Point blocked, Cell::STATE s)
{
    // Get values of indices passeed in
    int i = blocked.x;
    int j = blocked.y;
    int min;

    // If updated to block, set this cell's distance value to impassible
    if (s == Cell::BLOCKED)
        setDist(i, j, INT_MAX);
    // Otherwise
    else if (s == Cell::OPEN)
    {
        // Uh... this may be unnecessary
        min = checkNeighbors(blocked);
        setDist(i, j, min + 1);
    }

    // Begin updating dependent children
    std::queue<Point> dependents;

    // Enqueue children
    enqueueNeighbors(dependents, i, j);

    // Continue updating until there are no more left
    while(dependents.size() > 0)
    {
        // Get this point
        Point temp = dependents.front();
        i = temp.x;
        j = temp.y;

        min = checkNeighbors(temp); 

        // If this cell needs to be updated
        if (min != getDist(i, j) - 1 &&
            getDist(i, j) != INT_MAX && min != INT_MAX && 
            getDist(i, j) != 0)

        {
            setDist(i, j, min + 1);

            if (maxDist < min + 1)
                maxDist = min + 1;
            // Enqueue children
            enqueueNeighbors(dependents, i, j);
        }

        // Dequeue and move on
        dependents.pop();
    }
}

// Simple implementation of Bresenham's Line Algorithm to return
// a vector of points.
std::vector<Point> Map::lineAlgorithm(Point start, Point end)
{
    // Result vector
    std::vector<Point> line;

    // Get positions out of input
    int x0 = start.x;
    int y0 = start.y;
    int x1 = end.x;
    int y1 = end.y;

    // How far line will move in x direction
    float dx = x1 - x0;

    // How far line will move in y direction
    float dy = y1 - y0;

    // Accumulative error of pixellation
    float error = 0;

    // Sign of movement in x,y directions
    int xdir = ((x0 < x1) ? 1 : -1); 
    int ydir = ((y0 < y1) ? 1 : -1);

    // If not a vertical line
    if (dx != 0)
    {
        // How much error we accumulate by moving in the x direction
        float derr = std::abs((double) dy / dx);

        // Start with first y val
        int y = y0;

        // Get line starting from x0 to x1
        for (int x = x0; x != x1; x += xdir)
        {
            // Store point
            line.push_back(Point(x, y));

            // Update accumulated error
            error += derr;

            // If error rounds up
            while (error >= 0.5)
            {
                // Store point
                line.push_back(Point(x,y));

                // Move in the y direction
                y += ydir;

                // Error is now resolved.
                error = error - 1;
            }
        }
    }
    // Vertical line case. Nothing much to see here
    else
    {
        for (int y = y0; y != y1; y += ydir)
        {
            line.push_back(Point(x0, y));
        }
    }
    return line;
}

// Helper function that handles updating a line of cells to open
void Map::lineStates(std::vector<Point> points)
{
    int x, y;
    for (int i = 0; i < points.size(); i++)
    {
        x = points[i].x;
        y = points[i].y;
        if(x >= 0 && x < Nx &&
           y >= 0 && y < Ny)
            setState(x, y, Cell::OPEN);
    }
}

// Takes the start point of a ray and the end point of a ray and sets all
// points in between as unblocked and sets destination as blocked.
void Map::setBlocked(Point src, Point dst)
{
    // Get occupancy grid indices
    Point start = getIndex(src);
    Point end   = getIndex(dst);

    // Get the line from src to dst
    std::vector<Point> points = lineAlgorithm(start, end);
    
    // Update states in the line
    lineStates(points);

    // Set the end to blocked
    setState(end.x, end.y, Cell::BLOCKED);

    // Update the distances
    recalculate();
    //updateDistances(end, Cell::BLOCKED);
}


// Takes the start point of a ray and the end point of a ray and sets all
// points in between as unblocked and sets destination as open.
void Map::setOpen(Point src, Point dst)
{
    // Get occupancy grid indices
    Point start = getIndex(src);
    Point end   = getIndex(dst);

    // Get the line from src to dst
    std::vector<Point> points = lineAlgorithm(start, end);

    // Update states in the line
    lineStates(points);

    // Update the distances
    recalculate();
    //updateDistances(end, Cell::OPEN);
}

// Mutator function that allows user to set state at a cell
void Map::setState(int i, int j, Cell::STATE s) 
{
    if (i >= 0 && i < Nx &&
        j >= 0 && j < Nx)
    {
        map[Nx * j + i].state = s;
        if(s == Cell::BLOCKED)
            setDist(i, j, INT_MAX);
    }
}

// Mutator function that allows user to set distance at a cell
void Map::setDist(int i, int j, int d)         
{
    if (i >= 0 && i < Nx &&
        j >= 0 && j < Nx)
        map[Nx * j + i].dist  = d;
}

// TODO: Implement A*

void Map::AStar(Point p)
{
    // Initialize open and closed list
    std::list<idxDepth>::iterator it;
    std::list<idxDepth> open;
    std::list<idxDepth> closed;

    Point idx = getIndex(p);
    int x = idx.x;
    int y = idx.y;
    
    std::cout << "Running A*...\n";
    std::cout << "\t" << x << " " << y << "\n";

    // Check to make sure point is within bounds
    if (x < 0 || x >= Nx || y < 0 || y >= Ny)
        return;

    // Put starting node onto open list
    std::cout << "\tCreating new idxDepth(" << x << ", " << y << ", " << getDist(x, y) << ")\n";
    idxDepth * start = new idxDepth(x, y, getDist(x, y));
    std::cout << "\tSetting initial f val...\n";
    start->f = getDist(x, y);

    std::cout << "\tPushing first into list.\n";
    open.push_back(*start);

    std::cout << "\tFreeing start...?\n";
    // Free start
    free(start);

    // while open queue is not empty
    std::cout << "\tWhile queue is not empty...\n";
    while (!open.empty())
    {
        std::cout << "\t\tSorting list...\n";
        // Sort the list so that the lowest f is first
        open.sort();

        std::cout << "\t\tAllocating new idxDepth ";
        idxDepth * q = new idxDepth();
        *q = open.front();
        x = q->x;
        y = q->y;
        std::cout << x << " " << y << "\n";

        std::cout << "\t\tPopping list...\n";
        // Pop q off the open list
        open.pop_front();

        std::cout << "\t\tFor each successor...\n";
        // For each successor
        for (int j = y - 1; j < y + 2; j++)
        {
            for (int i = x - 1; i < x + 2; i++)
            {
                std::cout << "\t\t\t" << i << " " << j << "... ";
                if(i >= 0 && i < Nx && j >= 0 && j < Ny && (i != x || j != y))
                {
                    std::cout << "Is valid! Setting new idxDepth.\n";
                    idxDepth * s = new idxDepth(i, j, getDist(i, j));
                    s->parent = q;
                    
                    // If this is our goal
                    if (s->x == goal_idx.x && s->y == goal_idx.y)
                    {
                        std::cout << "Reached goal.\n";
                        indices.clear();
                        std::cout << "Pushing back initial point " << s->x << " " << s->y << "\n";
                        indices.push_back(Point(s->x, s->y));
                        std::cout << "Looking for parent...\n";
                        idxDepth * next = s;
                        std::cout << "While parent exists...\n";
                        while (next->parent)
                        {
                            std::cout << "Grabbed next parent\n";
                            next = next->parent;
                            std::cout << "Pushing back " << next->x << " " << next->y << "\n";
                            indices.push_back(Point(next->x, next->y));
                        }

                        std::cout << "No more parents.\n";
                        path.clear();
                        std::cout << "Translating to real world coordinates\n";
                        for (int n = indices.size() - 1; n >= 0; --n)
                        {
                            Point real = OccupancyToReal(indices[n]);
                            path.push_back(real);
                        }
                        std::cout << "Done! Returning...\n";
                        return;
                    }
                    
                    std::cout << "\t\t\tCalculating g... ";
                    float g = getDist(q->x, q->y) + 
                              sqrt((s->x - q->x) * (s->x - q->x) +
                                   (s->y - q->y) * (s->y - q->y)); 
                    std::cout << g << "\n\t\t\tCalculating h... ";
                    float h = sqrt((goal_idx.x - s->x) * (goal_idx.x - s->x) +
                                   (goal_idx.y - s->y) * (goal_idx.y - s->y));
                    std::cout << h << "\n\t\t\tSetting f... ";
                    s->f = g + h;
                    std::cout << s->f << "\n";

                    bool skip = false;

                    // Check if a node with the same position as successor is
                    // in open, with lower f value
                    std::cout << "\t\t\tCheck if a node with same position in open...\n";
                    for (it = open.begin(); it != open.end(); ++it)
                    {
                        if(it->x == s->x && it->y == s->y && it->f < s->f)
                            skip = true;
                    }
                    // Check if a node with the same position as successor is
                    // in closed, with lower f value
                    std::cout << "\t\t\tCheck if a node with same position in closed...\n";
                    for (it = closed.begin(); it != closed.end(); ++it)
                    {
                        if(it->x == s->x && it->y == s->y && it->f < s->f)
                            skip = true;
                    }

                    // Otherwise, add successor to open list
                    if(!skip)
                    {
                        std::cout << "\t\t\tSuccessor added.\n";
                        open.push_back(*s);
                    }
                }
                else
                    std::cout << "Not valid. Skipping\n";
            }
        }
    }
}

// Print out an image of the map we have
std::ostream& operator<<(std::ostream& out, Map m)
{
    out << "P3\n" << m.getNx() << " " << m.getNy() << "\n255\n";
    for (int j = m.getNy() - 1; j >= 0; j--)
    {
        for (int i = 0; i < m.getNx(); i++)
        {
            bool ispath = false;
            for (int k = 0; k < m.indices.size(); k++)
            {
                if (m.indices[k].x == i && m.indices[k].y == j)
                {
                    out << "255 0 0\n";
                    ispath = true;
                    break;
                }
            }
            if (!ispath)
            {
                if (m.map[j * m.getNx() + i].state == Cell::OPEN)
                {
                    out << "255 255 255\n";
                }
                else if (m.map[j * m.getNx() + i].state == Cell::BLOCKED)
                {
                    out << "0 0 0\n";
                }
                else
                {
                    int col = 255 * m.getDist(i, j) / (m.getMaxDist() + 1);
                    col = 255 - col;
                    //std::cout << col << "\n";
                    out << col << " " << col << " " << col << "\n";
                }
            }
        }
    }
    return out;
}
