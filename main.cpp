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

    m.setOpen(Point(0,0), Point(1.5,1.5));
    m.setBlocked(Point(0,0), Point(-1.0, 0.56));
    m.setBlocked(Point(0,0), Point(-1.0, 0.66));
    m.setBlocked(Point(0,0), Point(-1.0, 0.76));
    m.setBlocked(Point(0,0), Point(-1.0, 0.86));
    m.setBlocked(Point(0,0), Point(-1.0, 0.96));
    m.setBlocked(Point(0,0), Point(-0.90, 0.96));
    m.setBlocked(Point(0,0), Point(-0.80, 0.96));
    m.setBlocked(Point(0,0), Point(-0.70, 0.96));
    m.setBlocked(Point(0,0), Point(-0.60, 0.96));
    m.setBlocked(Point(0,0), Point(-0.50, 0.96));
    std::ofstream out;
    out.open("test.ppm");

    out << m;
    return 0;
}
