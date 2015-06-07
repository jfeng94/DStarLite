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

    m.getIndex(Point(0,0));
    m.getIndex(Point(1,0));
    m.getIndex(Point(0,1));
    m.getIndex(Point(-1.3,-0.3));
    m.getIndex(Point(1.42,-0.33));

    std::ofstream out;
    out.open("test.ppm");

    out << m;
    return 0;
}
