#ifndef MAP_H
#define	MAP_H
#include <iostream>
#include "gl_const.h"
#include <sstream>
#include <string>
#include <algorithm>
#include <vector>
class Map
{
    public:
        Map();
        Map(const Map& orig);
        ~Map();

        bool getMap(const std::vector<std::vector<int>>& map, int startx, int starty, int finishx, int finishy, int cell_size);
        bool CellIsTraversable (int i, int j) const;
        bool CellOnGrid (int i, int j) const;
        bool CellIsObstacle(int i, int j) const;
        int  getValue(int i, int j) const;

        int     height, width;
        int     start_i, start_j;
        int     goal_i, goal_j;
        double  cellSize;
        const std::vector<std::vector<int>> *Grid;
};

#endif
