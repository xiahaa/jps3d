#include "mission.h"
#include "astar.h"
#include "theta.h"
#include "gl_const.h"

Mission::Mission()
{
    search = nullptr;
    fileName = nullptr;
}

Mission::Mission(const char *FileName)
{
    fileName = FileName;
    search = nullptr;
}

Mission::~Mission()
{
    if (search)
        delete search;
}

bool Mission::getMap(int startX, int startY, int endX, int endY, int cellSize, std::vector<std::vector<int>> &mapData)
{
    return map.getMap(mapData, startX, startY, endX, endY, cellSize);
}

bool Mission::getConfig()
{
    return config.getConfig(fileName);
}

bool Mission::setDefaultConfig(bool use_theta)
{
    if (use_theta)
        config.setDefaultConfigTheta();
    else
        config.setDefaultConfigAstar();
    return true;
}

void Mission::createEnvironmentOptions()
{
    options = EnvironmentOptions(config.SearchParams[CN_SP_AS], config.SearchParams[CN_SP_AD],
                                     config.SearchParams[CN_SP_CC], config.SearchParams[CN_SP_MT]);
}

void Mission::createSearch()
{
    if (search)
        delete search;
    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_ASTAR)
    {
        std::cout << "Using A* search algorithm." << std::endl;
        search = new Astar(config.SearchParams[CN_SP_HW], config.SearchParams[CN_SP_BT]);
    }
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_TH)
    {
        std::cout << "Using Theta* search algorithm." << std::endl;
        search = new Theta(config.SearchParams[CN_SP_HW], config.SearchParams[CN_SP_BT]);
    }
}

void Mission::startSearch()
{
    sr = search->startSearch(map, options);
    if (config.SearchParams[CN_SP_PS])
    {
        smooth_search_result(sr, map, options.cutcorners);
    }
}

void Mission::getPath(std::vector<std::vector<int>> &path)
{
    path.clear();
    if (sr.pathfound) {
        std::list<Node> &srpath = *sr.lppath;
        for (std::list<Node>::const_iterator it = srpath.begin(); it != srpath.end(); it++) {
            std::vector<int> point;
            point.push_back(it->j);
            point.push_back(it->i);
            path.push_back(point);
        }
        // The sr.lppath should already contain all nodes from start to goal.
        // The following lines attempting to add sr.end are removed as SearchResult has no 'end' member
        // and lppath is expected to be complete.
        // std::vector<int> endPoint;
        // endPoint.push_back(sr.end.j);
        // endPoint.push_back(sr.end.i);
        // path.push_back(endPoint);
    }
}

bool Mission::getPathValid()
{
    return sr.pathfound;
}

void Mission::printSearchResultsToConsole()
{
    std::cout << "Path ";
    if (!sr.pathfound)
        std::cout << "NOT ";
    std::cout << "found!" << std::endl;
    std::cout << "numberofsteps=" << sr.numberofsteps << std::endl;
    std::cout << "nodescreated=" << sr.nodescreated << std::endl;
    if (sr.pathfound) {
        std::cout << "pathlength=" << sr.pathlength << std::endl;
        std::cout << "pathlength_scaled=" << sr.pathlength * map.cellSize << std::endl;
    }
    std::cout << "time=" << sr.time << std::endl;
}


const char *Mission::getAlgorithmName()
{
    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_ASTAR)
        return CNS_SP_ST_ASTAR;
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_TH)
        return CNS_SP_ST_TH;
    else
        return "";
}
