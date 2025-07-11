#ifndef CONFIG_H
#define	CONFIG_H
#include <string>


class Config
{
    public:
        Config();
        Config(const Config& orig);
        ~Config();
        bool getConfig(const char *FileName);

        // Set all search parameters directly (see gl_const.h for details)
        // st: search type (CN_SP_ST_ASTAR or CN_SP_ST_TH)
        // hw: heuristic weight (>=1)
        // mt: metric type (CN_SP_MT_DIAG, CN_SP_MT_MANH, CN_SP_MT_EUCL, CN_SP_MT_CHEB)
        // bt: breaking ties (CN_SP_BT_GMIN or CN_SP_BT_GMAX)
        // ad: allow diagonal (0 or 1)
        // cc: cut corners (0 or 1)
        // asq: allow squeeze (0 or 1)
        // ps: post smoothing (0 or 1)
        void setConfig(int st, double hw, int mt, int bt, int ad, int cc, int asq, int ps);

        // Set parameters by tag name (as in gl_const.h) and value (string or double)
        // Returns true if the tag is recognized and value is set, false otherwise
        bool setParamByTag(const std::string& tag, const std::string& value);
        bool setParamByTag(const std::string& tag, double value);

        // Reset all parameters to default values (A* with Euclidean, all options off)
        void setDefaultConfigAstar();
        void setDefaultConfigTheta();


    public:
        double*         SearchParams;
        unsigned int    N;

};

#endif
