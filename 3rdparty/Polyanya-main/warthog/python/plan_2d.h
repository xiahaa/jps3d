#ifndef WARTHOG_PLAN_2D_H
#define WARTHOG_PLAN_2D_H

#include <vector>
#include <string>
namespace warthog
{

int
plan_2d(std::vector<float> &origin, std::vector<int> &dim,
                std::vector<signed char> &map_data, std::vector<float> &start_f,
                std::vector<float> &goal_f, float resolution,
                std::vector<std::vector<double> > &path, double &time_spent,
                std::string &algorithm);

}

#endif