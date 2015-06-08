#include <vector>
#include <queue>
#include <iostream>
#include <math.h>
#include <cmath>
#include <limits.h>
#include <float.h>


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
    }
    idxDepth(int X, int Y, int d)
    {
        x = X;
        y = Y;
        f = FLT_MAX;
        depth = d;
    }
    idxDepth(Point p, int d)
    {
        x = p.x;
        y = p.y;
        f = FLT_MAX;
        depth = d;
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
    updateDistances(end, Cell::BLOCKED);
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
    updateDistances(end, Cell::OPEN);
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

// TODO: Implement A*

void Map::AStar(Point p)
{
    // A* Should take the robot's current position, and find the
    // occupancy grid world indices. Use getIndex.
    Point initial_p, occupancy_p, next_neighbor, neighbor_indices, real_neighbor;
    int finished = 0;
    printf("Initial Point: %f %f \n", p.x, p.y);
    indices.push_back(getIndex(p));    

    std::vector<idxDepth *> open;
    std::vector<idxDepth *> closed;

    std::vector<Point> path;

    path.push_back(p);
    open.push_back(new idxDepth(p, getDist(p.x, p.y)));

    while (!open.empty())
    {
        // Find node with least f on the open list, call it "q"
        // Simple iteration through vector
            float min = DBL_MAX;
            int q_i;
            for (int i = 0; i < open.size(); i++){
                if (open[i]->f < min){
                    min = open[i]->f;
                    q_i = i;
                }
            }
            idxDepth *q;
            *q = *open[q_i];
        // Pop q off open list. Use erase function.
            open.erase(open.begin() + q_i - 1);
        // Generate q's 8 successors and set their parents to q
        // Should write a helper function for this
        // For each successor
            
            // If successor is the goal, stop the search
            for (int m = q->y - 1; m < q->y + 2; m++){
                for (int n = q->x - 1; n < q->x + 2; n++){
                    idxDepth * successor;
                    successor->parent = q;
                    successor->x = n;
                    successor->y = m;
                    successor->depth = getDist(n, m);
                    if (successor->depth == 0)
                    {
                        return;
                        while (successor->parent)
                        {
                            idxDepth * temporary = successor->parent;
                            successor = temporary->parent;                        
                            path.push_back(Point(n, m));
                        }
                        return;
                    }
                    else{
                        successor->f = q->f + 
                                      sqrt((successor->x - q->x) * (successor->x - q->x) +
                                           (successor->y - q->y) * (successor->y - q->y)) +
                                      sqrt((successor->x - goal.x) * (successor->x - goal.x) +
                                           (successor->y - goal.y) * (successor->y - goal.y));

                        for (int i = 0; i < open.size(); i++){
                            if (open[i]->x == successor->x && open[i]->y == successor->y){
                                
                            }
                            if (closed[i]->x == successor->x && closed[i]->y == successor->y){
            
                            }
                            else{
                                open.push_back(successor);
                            }
                        }
                    }
                }
            }
            closed.push_back(q);
            // g = q.g + dist between sucessor and q
            
            // h = distance from goal to successor

            // f = g + h

            // If a node with the same position as successor is in then open list
            // which has a lower f than successor, skip

            // If a node with the same position as successor is in closed list
            // which has lower f than successor, skip

            // Else, add node to open
        //end

        // Push q onto closed list
    }
    
    // A* Should take the robot's current position, and find the
    // occupancy grid world indices. Use getIndex.
    
    // Look at your neighbors, find the direction you need to go to
    // The way I've written it, every point on the occupancy grid
    // is guaranteed to have a neighbor that's 1 unit closer than
    // the current point
    /*
    path.push_back(p);
    while (finished == 0)
    {
        // Get node we are examining
        initial_p = path[path.size() - 1];
        printf("Current Point: %f %f\n", initial_p.x, initial_p.y);

        // Get occupancy coordinates
        occupancy_p = getIndex(initial_p);
        printf("Occupancy Point: %f %f\n", occupancy_p.x, occupancy_p.y);
	      next_neighbor = neighborIndex(occupancy_p);
        indices.push_back(next_neighbor);
        printf("Next Neighbor: %f %f\n", next_neighbor.x, next_neighbor.y);
        // Store the occupancy grid coordinates into indices.
        neighbor_indices = OccupancyToReal(next_neighbor);
        printf("Neighbor Real Indices: %f %f\n", neighbor_indices.x, neighbor_indices.y);
        // After you're done transform the points back into real world
        // coordinates with getReal(int i, int j)
        

        // Store the transformed points into the path member
        path.push_back( neighbor_indices );

        // NOTE: Make sure you go all the way until you get a node with depth = 0.
        
        // Store the occupancy grid coordinates into indices.
        if (checkNeighbors(next_neighbor) == 0){
            finished = 1;
        }
        else{
            printf("Current Depth: %d\n", checkNeighbors(next_neighbor));
        }

        // After you're done transform the points back into real world
        // coordinates with getReal(int i, int j)


        // Store the transformed points into the path member

        for (int index = 0; index < indices.size(); index++)
        {
            std::cout << indices[index].x << " " << indices[index].y << "\n";
        }
    }*/
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
