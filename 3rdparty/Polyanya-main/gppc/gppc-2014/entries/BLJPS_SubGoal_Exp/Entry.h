
#pragma once
#include <stdint.h>
#include <vector>
#include "Coordinate.h"

void PreprocessMap(std::vector<bool> &bits, int width, int height, const char *filename);
void *PrepareForSearch(std::vector<bool> &bits, int width, int height, const char *filename);
bool GetPath(void *data, xyLoc s, xyLoc g, std::vector<xyLoc> &path);
const char *GetName();
