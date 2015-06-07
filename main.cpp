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

    std::ofstream out;
    out.open("test.ppm");

    out << m;
    return 0;
}
