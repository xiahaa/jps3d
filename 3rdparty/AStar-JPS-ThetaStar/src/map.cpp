#include "map.h"

Map::Map()
{
    height = -1;
    width = -1;
    start_i = -1;
    start_j = -1;
    goal_i = -1;
    goal_j = -1;
    Grid = nullptr;
    cellSize = 1;
}

Map::~Map()
{
}

bool Map::CellIsTraversable(int i, int j) const
{
    return ((*Grid)[i][j] == CN_GC_NOOBS);
}

bool Map::CellIsObstacle(int i, int j) const
{
    return ((*Grid)[i][j] != CN_GC_NOOBS);
}

bool Map::CellOnGrid(int i, int j) const
{
    return (i < height && i >= 0 && j < width && j >= 0);
}

bool Map::getMap(const std::vector<std::vector<int>>& map, int startx, int starty, int finishx, int finishy, int cell_size)
{
    Grid = &map;
    if (!Grid || Grid->empty() || (*Grid)[0].empty()) {
        std::cout << "Error! Empty map provided!" << std::endl;
        return false;
    }
    height = map.size();
    width = map[0].size();

    // printf("Map size: %d x %d\n", height, width);
    start_i = starty;
    start_j = startx;
    goal_i = finishy;
    goal_j = finishx;
    cellSize = cell_size;

    // printf("Start cell: (%d, %d)\n", start_i, start_j);
    // printf("Goal cell: (%d, %d)\n", goal_i, goal_j);
    // printf("Cell size: %f\n", cellSize);
    // printf("Start cell value: %d\n", (*Grid)[start_i][start_j]);
    // printf("Goal cell value: %d\n", (*Grid)[goal_i][goal_j]);
    // printf("CN_GC_NOOBS: %d\n", CN_GC_NOOBS);

    if ((*Grid)[start_i][start_j] != CN_GC_NOOBS) {
        std::cout << "Error! Start cell is not traversable (cell's value is" << (*Grid)[start_i][start_j] << ")!"
                  << std::endl;
        return false;
    }

    if ((*Grid)[goal_i][goal_j] != CN_GC_NOOBS) {
        std::cout << "Error! Goal cell is not traversable (cell's value is" << (*Grid)[goal_i][goal_j] << ")!"
                  << std::endl;
        return false;
    }

    return true;
}

int Map::getValue(int i, int j) const
{
    if (i < 0 || i >= height)
        return -1;

    if (j < 0 || j >= width)
        return -1;

    return (*Grid)[i][j];
}
