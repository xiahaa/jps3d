/**
 * Helper program to extract first move matrix for a given goal from C++ CPD.
 * This can be called from Python to compare with Python implementation.
 */

#include <stdio.h>
#include <cstdlib>
#include <vector>
#include "Entry.h"
#include "cpd.h"
#include "mapper.h"
#include "adj_graph.h"

// State structure (defined in Entry.cpp, but we need it here)
struct State{
	CPD cpd;
	Mapper mapper;
	AdjGraph graph;
	int current_node;
	int target_node;
};

// LoadMap function (static to avoid conflicts with main.cpp)
static void LoadMap(const char *fname, std::vector<bool> &map, int &width, int &height);

int main(int argc, char **argv)
{
    if (argc != 5)
    {
        printf("Usage: %s <preprocessed_file> <map_file> <goal_x> <goal_y>\n", argv[0]);
        printf("Outputs first move matrix as binary: width, height, then matrix data\n");
        return 1;
    }

    const char *preprocessed_file = argv[1];
    const char *map_file = argv[2];
    int goal_x = atoi(argv[3]);
    int goal_y = atoi(argv[4]);

    // Load map
    std::vector<bool> mapData;
    int width, height;
    LoadMap(map_file, mapData, width, height);

    // Prepare search state
    void *state = PrepareForSearch(mapData, width, height, preprocessed_file);
    if (state == nullptr)
    {
        fprintf(stderr, "Failed to load preprocessed data\n");
        return 1;
    }

    State *s = static_cast<State *>(state);

    // Create output matrix (same size as map, -1 for obstacles, -2 for unreachable, direction for others)
    std::vector<int> matrix(width * height, -2); // Initialize to unreachable

    // Mark obstacles
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            if (!mapData[y * width + x])
            {
                matrix[y * width + x] = -1; // Obstacle
            }
        }
    }

    // Get goal node
    xyLoc goal_loc;
    goal_loc.x = goal_x;
    goal_loc.y = goal_y;
    int goal_node = s->mapper(goal_loc);

    if (goal_node == -1)
    {
        fprintf(stderr, "Goal position is an obstacle or out of bounds\n");
        return 1;
    }

    // For each free cell, get first move to goal
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            if (!mapData[y * width + x])
                continue; // Skip obstacles

            xyLoc start_loc;
            start_loc.x = x;
            start_loc.y = y;
            int start_node = s->mapper(start_loc);

            if (start_node == -1)
                continue; // Should not happen for free cells

            if (start_node == goal_node)
            {
                matrix[y * width + x] = 0; // At goal
                continue;
            }

            // Get first move from CPD
            unsigned char first_move = s->cpd.get_first_move(start_node, goal_node);

            if (first_move == 0xF)
            {
                matrix[y * width + x] = -2; // Unreachable
            }
            else
            {
                matrix[y * width + x] = first_move;
            }
        }
    }

    // Output matrix: width (int), height (int), then matrix data (int array)
    fwrite(&width, sizeof(int), 1, stdout);
    fwrite(&height, sizeof(int), 1, stdout);
    fwrite(matrix.data(), sizeof(int), width * height, stdout);

    return 0;
}

static void LoadMap(const char *fname, std::vector<bool> &map, int &width, int &height)
{
    FILE *f;
    f = fopen(fname, "r");
    if (f)
    {
        fscanf(f, "type octile\nheight %d\nwidth %d\nmap\n", &height, &width);
        map.resize(height * width);
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                char c;
                do
                {
                    fscanf(f, "%c", &c);
                } while (isspace(c));
                map[y * width + x] = (c == '.' || c == 'G' || c == 'S');
            }
        }
        fclose(f);
    }
}
