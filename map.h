#ifndef MAP_H
#define MAP_H

#include <vector>
#include <int.h>


struct Point
{
    // Data members
    float x, y;

    // Constructor
    Point() : x(0), y(0) {};
    Point(float X, float Y) {x = X; y = Y;}

    // Operator overrides
    Point operator+ (Point p) {return Point(x + p.x, y + p.y);}
    Point operator- (Point p) {return Point(x + p.x, y + p.y);}
    void  operator+=(Point p) {x += p.x;}
    void  operator-=(Point p) {y += p.y;}
    Point operator= (Point p) {x = p.x; y = p.x; return p;};
};

struct Cell
{
    // Enumeration for status
    enum STATE
    {
        OPEN,
        BLOCKED,
        UNKNOWN
    };

    // Data members
    Point p;
    int xidx, yidx;
    int dist;
    STATE state;

    // Constructors
    Cell(float x, float y, int i, int j) 
    {
        p.x   = x;
        p.y   = y;
        xidx  = i;
        yidx  = j;
        dist  = INT_MAX;
        state = UNKNOWN;
    }
    Cell(Point a, int i, int j)
    {
        p     = a;
        xidx  = i;
        yidx  = j;
        dist  = INT_MAX;
        state = UNKNOWN;
    }
};

class Map
{
    private:
        // Data members
        std::vector<Cell> map;
        float xmin, xmax, ymin, ymax;
        float res;
        int Nx, Ny;

    public:
        // Constructor
        Map(float, float, float, float, float);

        // Initialize map
        void init(Point p);
        void initHelper(int i, int j, int depth);

        Point getIndex(Point);

        // Mutators
        void setState(int i, int j, Cell::STATE s) {map[Nx * j + i].state = s;}
        void setDist (int i, int j, int d)         {map[Nx * j + i].dist  = d;}

        // Accessors
        int getNx() {return Nx;}
        int getNy() {return Ny;}
        Cell get(int i, int j) {return map[Nx * i + j];}

        // Operator overrider
        friend std::ostream& operator<<(std::ostream&, Map); 

};
#endif
