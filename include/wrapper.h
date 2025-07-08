#pragma once
#include <vector>


int plan_2d(std::vector<float> &origin, std::vector<int> &dim, std::vector<signed char> &map, std::vector<float> &start, std::vector<float> &goal, float resolution, std::vector<std::vector<double> > &path, double &time_spent, bool use_jps);