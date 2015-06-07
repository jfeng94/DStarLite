#include <vector>
#include <iostream>
#include <math.h>


#include "map.h"

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

Point Map::getIndex(Point p)
{
    int i = (int) round((p.x - xmin) / res);
    int j = (int) round((p.y - ymin) / res);

    //std::cout << "Index: " << i << " " << j << "\n";
    //std::cout << map[j * Nx + i].p.x << " " << map[j * Nx + i].p.y << "\n";
    //setState(i, j, Cell::BLOCKED);
    return Point(i, j);
}

void Map::init(Point p)
{
    Point idx = getIndex(p);
    int i = idx.x;
    int j = idx.y;
    
    setDist(i, j, 0);

    // Set distance of surrounding points
    initHelper(i - 1, j - 1, 1); 
    initHelper(i - 1, j    , 1); 
    initHelper(i - 1, j + 1, 1); 
    initHelper(i    , j - 1, 1); 
    initHelper(i    , j + 1, 1); 
    initHelper(i + 1, j - 1, 1); 
    initHelper(i + 1, j    , 1); 
    initHelper(i + 1, j + 1, 1); 

}

void Map::initHelper(int i, int j, int depth)
{
    if (get(i,j).dist > depth)
    {
        
    }
}

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
            out << "175 175 175\n";
        }
    }
    return out;
}
