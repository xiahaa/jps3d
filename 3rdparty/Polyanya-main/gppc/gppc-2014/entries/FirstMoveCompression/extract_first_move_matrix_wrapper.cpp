/**
 * Pybind11 wrapper for extract_first_move_matrix
 * This allows calling the C++ first move matrix extraction from Python
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <vector>
#include <string>
#include <stdexcept>
#include <cstdio>
#include "Entry.h"
#include "cpd.h"
#include "mapper.h"
#include "adj_graph.h"

namespace py = pybind11;

// State structure (defined in Entry.cpp, but we need it here)
struct State{
	CPD cpd;
	Mapper mapper;
	AdjGraph graph;
	int current_node;
	int target_node;
};

// LoadMap function (static to avoid conflicts with main.cpp)
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

/**
 * Extract first move matrix from C++ CPD for a given goal.
 *
 * @param preprocessed_file Path to preprocessed C++ data file
 * @param map_file Path to .map file
 * @param goal_x Goal x coordinate
 * @param goal_y Goal y coordinate
 * @return 2D numpy array: -1 for obstacles, -2 for unreachable, direction code (0-7) for valid moves
 */
py::array_t<int32_t> extract_first_move_matrix_cpp(
    const std::string &preprocessed_file,
    const std::string &map_file,
    int goal_x,
    int goal_y
)
{
    // Check if preprocessed file exists
    FILE *f = fopen(preprocessed_file.c_str(), "rb");
    if (f == nullptr)
    {
        throw std::runtime_error("Preprocessed file does not exist: " + preprocessed_file);
    }
    fclose(f);

    // Load map
    std::vector<bool> mapData;
    int width, height;
    LoadMap(map_file.c_str(), mapData, width, height);

    if (mapData.empty())
    {
        throw std::runtime_error("Failed to load map file: " + map_file);
    }

    // Prepare search state
    void *state = PrepareForSearch(mapData, width, height, preprocessed_file.c_str());
    if (state == nullptr)
    {
        throw std::runtime_error("Failed to load preprocessed data from: " + preprocessed_file);
    }

    State *s = static_cast<State *>(state);

    // Create output matrix (same size as map, -1 for obstacles, -2 for unreachable, direction for others)
    // Use the same encoding as Python: -1=obstacle, -2=unreachable, 0-7=direction
    std::vector<int32_t> matrix_data(width * height, -2); // Initialize to unreachable

    // Mark obstacles
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            if (!mapData[y * width + x])
            {
                matrix_data[y * width + x] = -1; // Obstacle
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
        throw std::runtime_error("Goal position is an obstacle or out of bounds");
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
                matrix_data[y * width + x] = 0; // At goal (same as Python)
                continue;
            }

            // Get first move from CPD
            unsigned char first_move = s->cpd.get_first_move(start_node, goal_node);

            if (first_move == 0xF)
            {
                matrix_data[y * width + x] = -2; // Unreachable
            }
            else
            {
                // C++ returns direction index (0-15), but Python uses 0-3 for 4-connected or 0-7 for 8-connected
                // The C++ code uses the same direction encoding as the graph, which should match
                // For now, we'll return the value as-is and let the comparison handle any mapping
                matrix_data[y * width + x] = first_move;
            }
        }
    }

    // Create numpy array (height x width, row-major order like Python)
    py::array_t<int32_t> result({height, width});
    auto result_buf = result.mutable_unchecked<2>();

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            result_buf(y, x) = matrix_data[y * width + x];
        }
    }

    return result;
}

PYBIND11_MODULE(cpp_first_move_matrix, m)
{
    m.doc() = "C++ First Move Matrix Extraction using pybind11";

    m.def("extract_first_move_matrix", &extract_first_move_matrix_cpp,
          "Extract first move matrix from C++ CPD",
          py::arg("preprocessed_file"),
          py::arg("map_file"),
          py::arg("goal_x"),
          py::arg("goal_y"));
}
