#include "Entry.h"
#include "Timer.h"

#include <sys/stat.h>

bool FileExists(const char* filename) {
    struct stat buffer;
    return (stat(filename, &buffer) == 0);
}

int plan_2d(std::vector<float> &origin, std::vector<int> &dim,
std::vector<signed char> &map_data, std::vector<float> &start_f,
std::vector<float> &goal_f, float resolution,
std::vector<std::vector<double> > &path, double &time_spent)
{
    std::vector<bool> mapData;
    for(auto d:map_data)
    {
        mapData.push_back(d==0);
    }
    int width = dim[0];
    int height = dim[1];
    std::string filename = "./map.txt";
    // if file not exists, preprocess the map
    if(!FileExists(filename.c_str()))
    {
        printf("Preprocessing map to %s\n", filename.c_str());
        PreprocessMap(mapData, width, height, filename.c_str());
    }
    void *reference = PrepareForSearch(mapData, width, height, filename.c_str());

    std::vector<xyLoc> thePath;
    Timer t;
    xyLoc s, g;
    printf(__FILE__ ":%d: Planning from (%f, %f) to (%f, %f)\n", __LINE__, start_f[0], start_f[1], goal_f[0], goal_f[1]);
    s.x = int((start_f[0]-origin[0])/resolution);
    s.y = int((start_f[1]-origin[1])/resolution);
    g.x = int((goal_f[0]-origin[0])/resolution);
    g.y = int((goal_f[1]-origin[1])/resolution);
    printf(__FILE__ ":%d: Planning from (%f, %f) to (%f, %f)\n", __LINE__, start_f[0], start_f[1], goal_f[0], goal_f[1]);

    t.StartTimer();
    printf("Planning from (%f, %f) to (%f, %f)\n", start_f[0], start_f[1], goal_f[0], goal_f[1]);
    bool done = GetPath(reference, s, g, thePath);
    t.EndTimer();
    printf("Planning done, found path: %d\n", done);
    time_spent = t.GetElapsedTime() * 1000.0; // Convert to milliseconds

    for(auto p:thePath)
    {
        std::vector<double> point;
        point.push_back(p.x*resolution+origin[0]);
        point.push_back(p.y*resolution+origin[1]);
        path.push_back(point);
    }
    return 0;
}