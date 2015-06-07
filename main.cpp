#include "map.h"

#include <fstream>
#include <iostream>

int main(int argc, char** argv)
{
    float xmin = atof(argv[1]);
    float ymin = atof(argv[2]);
    float xmax = atof(argv[3]);
    float ymax = atof(argv[4]);
    float res  = atof(argv[5]);

    Map m(xmin, ymin, xmax, ymax, res);

    Point p(0,0);
    
    m.init(p);

    // Setting open ranges 
    m.setOpen(Point(0,0), Point(1.5,1.5));

    // Setting blocked ranges
    m.setBlocked(Point(0,0), Point(-1.0, 0.56));
    m.setBlocked(Point(0,0), Point(-1.0, 0.66));
    m.setBlocked(Point(0,0), Point(-1.0, 0.76));
    m.setBlocked(Point(0,0), Point(-1.0, 0.86));
    m.setBlocked(Point(0,0), Point(-1.0, 0.96));
    m.setBlocked(Point(0,0), Point(-0.9, 0.96));
    m.setBlocked(Point(0,0), Point(-0.8, 0.96));
    m.setBlocked(Point(0,0), Point(-0.7, 0.96));
    m.setBlocked(Point(0,0), Point(-0.6, 0.96));
    m.setBlocked(Point(0,0), Point(-0.5, 0.96));


    // Example of how to print out occupancy matrix
    std::ofstream out;
    out.open("test.ppm");
    out << m;


    return 0;
}
