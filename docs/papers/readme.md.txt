# todo
1. try github opensource implementation, no need to fully understand the algorithm as some of them are almost de-facto solution.

2. benchmark on navmesh generation using nanomesh and polyanya：https://github.com/hpgem/nanomesh，https://www.geom.at/boolean-operations/，
更复杂的就是用ctypes去调用recastnavigation了

iros astar：我自己的实现，没有发现很明显的效果。

visibility

hog2极其复杂


# reference 
1. https://arongranberg.com/astar/documentation/beta/hierarchicalgraph.html
2. C# HPA: https://github.com/hugoscurti/hierarchical-pathfinding，如果没有C版本，可以考虑使用agent转换成C++版本
3. https://github.com/narsue/BLJPS_Python/tree/master: fastest one up to now
4. https://github.com/narsue/Dynamic_BLJPS: 有针对动态地图的接口
5. https://github.com/SandSnip3r/Pathfinder：有CDT 构建和path过程，但是CDT如何动态更新，没有说明；
6. 用navmesh来做路径规划，但是没有解决怎么dynamicly更新navmesh. https://github.com/Tugcga/Path-Finder/tree/main
7. https://github.com/lucho1/JumpPointSearch/tree/master: 很好的path planning的tutorial。
8. 一个C++版本的HPAstar：https://github.com/r-silveira/hpaStar
9. Daniel Harabor：https://bitbucket.org/dharabor/pathfinding/src/master/warthog/
10. https://www.movingai.com/SAS/
11. https://github.com/xiahaa/Polyanya/tree/main/gppc/gppc-2014/entries：还有几个没有完全试完。
12. https://gamedev.stackexchange.com/questions/203367/creating-a-nav-mesh
13. https://github.com/nathansttt/hog2/tree/PDB-refactor


# VG reference
1. https://karlobermeyer.github.io/VisiLibity1/

# GAN/TransPath
1. https://github.com/lucho1/JumpPointSearch/tree/master
2. https://github.com/AIRI-Institute/TransPath/tree/main

