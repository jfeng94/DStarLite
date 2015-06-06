#include <vector>

struct Point
{
    // Data members
    float x, y;

    // Constructor
    Point::Point(float X, float Y) {x = X; y = Y;}

    // Operator overrides
    Point operator+ (Point p) {Point(x + p.x, y + p.y);}
    Point operator- (Point p) {Point(x + p.x, y + p.y);}
    void  operator+=(Point p) {x += p.x;}
    void  operator-=(Point p) {y += p.y;}
    Point operator= (Point p) {x = p.x; y = p.x; return p};
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
    STATE state(UNKNOWN);

    // Constructors
    Cell(float x, float y, int i, int j) 
    {
        p.x = x;
        p.y = y;
        xidx = i;
        yidx = j;
    }
    Cell(Point a, int i, int j)
    {
        p = a;
        xidx = i;
        yidx = j;
    }

    // Mutators
    void setState(STATE s) {state = s;}
    void setDist (int d)   {dist  = d;}
};

class Map
{
    private:
        // Data members
        Cell ** map;
        float xmin, xmax, ymin, ymax;
        float res;

    public:
        // Constructor
        Map(float, float, float, float, float);

        // Initialize map
        void init(Point p);

        // 
        friend ostream& operator<<(ostream&, Map); 

};
