#ifndef GL_CONST_H
#define	GL_CONST_H

#define CN_PI_CONSTANT 3.14159265359
#define CN_SQRT_TWO    1.41421356237

//XML tags
#define CNS_TAG_ROOT "root"

    #define CNS_TAG_ALG             "algorithm"
        #define CNS_TAG_ST          "searchtype"
        #define CNS_TAG_HW          "hweight"
        #define CNS_TAG_MT          "metrictype"
        #define CNS_TAG_BT          "breakingties"
        #define CNS_TAG_AS          "allowsqueeze"
        #define CNS_TAG_AD          "allowdiagonal"
        #define CNS_TAG_CC          "cutcorners"
        #define CNS_TAG_PS          "postsmoothing"


//Search Parameters
    #define CN_SP_ST 0
        #define CNS_SP_ST_ASTAR         "astar"
        #define CNS_SP_ST_TH            "theta"

        #define CN_SP_ST_ASTAR          2
        #define CN_SP_ST_TH             4

    #define CN_SP_AD 1 //AllowDiagonal

    #define CN_SP_CC 2 //CutCorners

    #define CN_SP_AS 3 //AllowSqueeze

    #define CN_SP_MT 4 //MetricType

        #define CNS_SP_MT_DIAG  "diagonal"
        #define CNS_SP_MT_MANH  "manhattan"
        #define CNS_SP_MT_EUCL  "euclid"
        #define CNS_SP_MT_CHEB  "chebyshev"

        #define CN_SP_MT_DIAG   0
        #define CN_SP_MT_MANH   1
        #define CN_SP_MT_EUCL   2
        #define CN_SP_MT_CHEB   3

    #define CN_SP_HW 5 //HeuristicWeight

    #define CN_SP_BT 6 //BreakingTies

        #define CNS_SP_BT_GMIN "g-min"
        #define CNS_SP_BT_GMAX "g-max"

        #define CN_SP_BT_GMIN 0
        #define CN_SP_BT_GMAX 1

    #define CN_SP_PS 7 //PostSmoothing



//Grid Cell
    #define CN_GC_NOOBS 0
    #define CN_GC_OBS   1

#endif
