#ifndef MISSION_H
#define	MISSION_H

#include "map.h"
#include "config.h"
#include "isearch.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include "astar.h"
#include "theta.h"
#include "path_smoothing.h"

class Mission
{
    public:
        Mission();
        Mission (const char* fileName);
        ~Mission();

        bool getMap(int startX, int startY, int endX, int endY, int cellSize, std::vector<std::vector<int>> &mapData);
        bool getConfig();
        void createSearch();
        void createEnvironmentOptions();
        void startSearch();
        void printSearchResultsToConsole();
        bool setDefaultConfig(bool use_theta);
        void getPath(std::vector<std::vector<int>> &path);
        bool getPathValid();

    private:
        const char* getAlgorithmName();

        Map                     map;
        Config                  config;
        EnvironmentOptions      options;
        ISearch*                search;
        const char*             fileName;
        SearchResult            sr;
};

#endif
