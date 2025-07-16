#include "Entry.h"
#include <chrono>
#include <iostream>

int plan_2d(std::vector<float> &origin, std::vector<int> &dim,
std::vector<signed char> &map_data, std::vector<float> &start_f,
std::vector<float> &goal_f, float resolution,
std::vector<std::vector<double> > &path, double &time_spent)
{
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::vector<bool> mapData;
    for(auto d:map_data)
    {
        mapData.push_back(d==0);
    }
    int width = dim[0];
    int height = dim[1];

    void *reference = PrepareForSearch(mapData, width, height, "");

    std::vector<xyLoc> thePath;
    xyLoc s, g;
    s.x = int((start_f[0]-origin[0])/resolution);
    s.y = int((start_f[1]-origin[1])/resolution);
    g.x = int((goal_f[0]-origin[0])/resolution);
    g.y = int((goal_f[1]-origin[1])/resolution);

    bool done = GetPath(reference, s, g, thePath);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    time_spent = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    for(auto p:thePath)
    {
        std::vector<double> point;
        point.push_back(p.x*resolution+origin[0]);
        point.push_back(p.y*resolution+origin[1]);
        path.push_back(point);
    }
    if (done)
    {
        path.push_back({goal_f[0], goal_f[1]});
        std::cout << "Path found!" << std::endl;
    }
    return 0;
}