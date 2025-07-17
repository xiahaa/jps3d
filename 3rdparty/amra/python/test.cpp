// project includes
#include <amra/interface.h>
#include <amra/constants.hpp>
#include <amra/helpers.hpp>

// standard includes
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>

int main(int argc, char** argv)
{
    std::string mapfile;
    if (argc > 1) {
        mapfile = argv[1];
    } else {
        mapfile = "dat/Boston_0_1024.map";
    }

    std::ifstream file(mapfile);
    if (!file.is_open()) {
        std::cerr << "Could not open map file: " << mapfile << std::endl;
        return 1;
    }

    std::string line, word;
    int width, height;

    std::getline(file, line); // type octile
    std::getline(file, line); // height
    std::stringstream ss(line);
    ss >> word >> height;
    std::getline(file, line); // width
    ss.clear();
    ss.str(line);
    ss >> word >> width;
    std::getline(file, line); // map

    std::vector<signed char> map_data;
    map_data.reserve(width * height);

    for (int r = 0; r < height; ++r) {
        std::getline(file, line);
        for (int c = 0; c < width; ++c) {
            auto itr = AMRA::MOVINGAI_DICT.find(line[c]);
            if (itr != AMRA::MOVINGAI_DICT.end()) {
                map_data.push_back(itr->second);
            } else {
                int val = int(line[c]) - int('a') + 1;
                map_data.push_back(val * 10);
            }
        }
    }

    std::vector<float> origin = {0.0f, 0.0f};
    std::vector<int> dim = {width, height};
    float resolution = 1.0f;

    std::vector<float> start_f = {28.0f, 12.0f};
    std::vector<float> goal_f = {5.0f, 45.0f};

    std::vector<std::vector<double>> path;
    double time_spent;

    int result = plan_2d(origin, dim, map_data, start_f, goal_f, resolution, path, time_spent);

    if (result == 1) {
        std::cout << "Path found in " << time_spent << " seconds." << std::endl;
        // for (const auto& p : path) {
        //     std::cout << p[0] << ", " << p[1] << std::endl;
        // }
    } else {
        std::cout << "Failed to find a path." << std::endl;
    }

    return 0;
}