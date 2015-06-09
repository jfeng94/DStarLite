#include "map.h"
#include <fstream>
#include <iostream>
#include <cstdlib>

int main(int argc, char ** argv)
{
    float xmin = atof(argv[1]);
    float ymin = atof(argv[2]);
    float xmax = atof(argv[3]);
    float ymax = atof(argv[4]);
    float res  = atof(argv[5]);

    Map m(xmin, ymin, xmax, ymax, res);
    Point p(0,0);
    m.init(p);

    m.setBlocked(Point(0,0), Point(2.0, 0.96));
    m.setBlocked(Point(0,0), Point(1.9, 0.96));
    m.setBlocked(Point(0,0), Point(1.8, 0.96));
    m.setBlocked(Point(0,0), Point(1.7, 0.96));
    m.setBlocked(Point(0,0), Point(1.6, 0.96));
    m.setBlocked(Point(0,0), Point(1.5, 0.96));
    m.setBlocked(Point(0,0), Point(1.4, 0.96));
    m.setBlocked(Point(0,0), Point(1.3, 0.96));
    m.setBlocked(Point(0,0), Point(1.2, 0.96));
    m.setBlocked(Point(0,0), Point(1.1, 0.96));
    m.setBlocked(Point(0,0), Point(0.9, 0.96));
    m.setBlocked(Point(0,0), Point(0.8, 0.96));
    m.setBlocked(Point(0,0), Point(0.7, 0.96));
    m.setBlocked(Point(0,0), Point(0.6, 0.96));
    m.setBlocked(Point(0,0), Point(0.5, 0.96));
    m.setBlocked(Point(0,0), Point(0.4, 0.96));
    m.setBlocked(Point(0,0), Point(0.3, 0.96));
    m.setBlocked(Point(0,0), Point(0.2, 0.96));
    m.setBlocked(Point(0,0), Point(0.1, 0.96));
    m.setBlocked(Point(0,0), Point(0.0, 0.96));
    m.setBlocked(Point(0,0), Point(-0.1, 0.96));
    m.setBlocked(Point(0,0), Point(-0.2, 0.96));
    m.setBlocked(Point(0,0), Point(-0.3, 0.96));
    m.setBlocked(Point(0,0), Point(-0.4, 0.96));
    m.setBlocked(Point(0,0), Point(-0.5, 0.96));
    m.setBlocked(Point(0,0), Point(-0.6, 0.96));
    m.setBlocked(Point(0,0), Point(-0.7, 0.96));
    m.setBlocked(Point(0,0), Point(-0.8, 0.96));
    m.setBlocked(Point(0,0), Point(-0.9, 0.96));
    m.setBlocked(Point(0,0), Point(-1.0, 0.96));
    m.setBlocked(Point(0,0), Point(-1.0, 0.86));
    m.setBlocked(Point(0,0), Point(-1.0, 0.76));
    m.setBlocked(Point(0,0), Point(-1.0, 0.66));
    m.setBlocked(Point(0,0), Point(-1.0, 0.56));
    m.setBlocked(Point(0,0), Point(-1.0, 0.46));
    m.setBlocked(Point(0,0), Point(-1.0, 0.36));
    m.setBlocked(Point(0,0), Point(-1.0, 0.26));
    m.setBlocked(Point(0,0), Point(-1.0, 0.16));
    m.setBlocked(Point(0,0), Point(-1.0, 0.06));
    m.setBlocked(Point(0,0), Point(-1.0, 0.00));
    m.setBlocked(Point(0,0), Point(-1.0,-0.10));
    m.setBlocked(Point(0,0), Point(-1.0,-0.20));
    m.setBlocked(Point(0,0), Point(-1.0,-0.30));
    m.setBlocked(Point(0,0), Point(-1.0,-0.40));
    m.setBlocked(Point(0,0), Point(-1.0,-0.50));
    m.setBlocked(Point(0,0), Point(-1.0,-0.60));
    m.setBlocked(Point(0,0), Point(-1.0,-0.70));
    m.setBlocked(Point(0,0), Point(-1.0,-0.80));
    m.setBlocked(Point(0,0), Point(-1.0,-0.90));
    m.setBlocked(Point(0,0), Point(-1.0,-1.00));
    m.setBlocked(Point(0,0), Point(-1.0,-1.10));
    m.setBlocked(Point(0,0), Point(-1.0,-1.20));
    m.setBlocked(Point(0,0), Point(-1.0,-1.30));
    m.setBlocked(Point(0,0), Point(-1.0,-1.40));
    m.setBlocked(Point(0,0), Point(-1.0,-1.50));
    m.setBlocked(Point(0,0), Point(-1.0,-1.60));
    m.setBlocked(Point(0,0), Point(-1.0,-1.70));

    m.setBlocked(Point(0,0), Point(-0.5, 0.56));
    m.setBlocked(Point(0,0), Point(-0.5, 0.46));
    m.setBlocked(Point(0,0), Point(-0.5, 0.36));
    m.setBlocked(Point(0,0), Point(-0.5, 0.26));
    m.setBlocked(Point(0,0), Point(-0.5, 0.16));
    m.setBlocked(Point(0,0), Point(-0.5, 0.06));
    m.setBlocked(Point(0,0), Point(-0.5, 0.00));
    m.setBlocked(Point(0,0), Point(-0.5,-0.10));
    m.setBlocked(Point(0,0), Point(-0.5,-0.20));
    m.setBlocked(Point(0,0), Point(-0.5,-0.30));
    m.setBlocked(Point(0,0), Point(-0.5,-0.40));
    m.setBlocked(Point(0,0), Point(-0.5,-0.50));
    m.setBlocked(Point(0,0), Point(-0.5,-0.60));
    m.setBlocked(Point(0,0), Point(-0.5,-0.70));
    m.setBlocked(Point(0,0), Point(-0.5,-0.80));
    m.setBlocked(Point(0,0), Point(-0.5,-0.90));
    m.setBlocked(Point(0,0), Point(-0.5,-1.00));
    m.setBlocked(Point(0,0), Point(-0.5,-1.10));
    m.setBlocked(Point(0,0), Point(-0.5,-1.20));
    m.setBlocked(Point(0,0), Point(-0.5,-1.30));
    m.setBlocked(Point(0,0), Point(-0.5,-1.40));
    m.setBlocked(Point(0,0), Point(-0.5,-1.50));
    m.setBlocked(Point(0,0), Point(-0.5,-1.60));
    m.setBlocked(Point(0,0), Point(-0.5,-1.70));
    m.setBlocked(Point(0,0), Point(-0.5,-1.80));
    m.setBlocked(Point(0,0), Point(-0.5,-1.90));
    m.setBlocked(Point(0,0), Point(-0.5,-2.00));

    m.setBlocked(Point(0,0), Point( 1.6, 0.6));
    m.setBlocked(Point(0,0), Point( 1.5, 0.6));
    m.setBlocked(Point(0,0), Point( 1.4, 0.6));
    m.setBlocked(Point(0,0), Point( 1.3, 0.6));
    m.setBlocked(Point(0,0), Point( 1.2, 0.6));
    m.setBlocked(Point(0,0), Point( 1.1, 0.6));
    m.setBlocked(Point(0,0), Point( 0.9, 0.6));
    m.setBlocked(Point(0,0), Point( 0.8, 0.6));
    m.setBlocked(Point(0,0), Point( 0.7, 0.6));
    m.setBlocked(Point(0,0), Point( 0.6, 0.6));
    m.setBlocked(Point(0,0), Point( 0.5, 0.6));
    m.setBlocked(Point(0,0), Point( 0.4, 0.6));
    m.setBlocked(Point(0,0), Point( 0.3, 0.6));
    m.setBlocked(Point(0,0), Point( 0.2, 0.6));
    m.setBlocked(Point(0,0), Point( 0.1, 0.6));
    m.setBlocked(Point(0,0), Point( 0.0, 0.6));
    m.setBlocked(Point(0,0), Point(-0.1, 0.6));
    m.setBlocked(Point(0,0), Point(-0.2, 0.6));
    m.setBlocked(Point(0,0), Point(-0.3, 0.6));
    m.setBlocked(Point(0,0), Point(-0.4, 0.6));
    m.setBlocked(Point(0,0), Point(-0.5, 0.6));
    m.setBlocked(Point(0,0), Point(-0.6, 0.6));
    m.setBlocked(Point(0,0), Point(-0.7, 0.6));
    m.setBlocked(Point(0,0), Point(-0.8, 0.6));
    m.setBlocked(Point(0,0), Point(-0.9, 0.6));
    m.setBlocked(Point(0,0), Point(-1.0, 0.6));

    Point end(-1.9, 1.9);
    m.AStar(end);
    
    std::vector<Point> path = m.getPath();

    for (int i = 0; i < path.size(); ++i)
    {
        std::cout << path[i].x << " " << path[i].y << "\n";
    }
    std::ofstream out;
    out.open("test.ppm");
    out << m;


    return 0;

}
