#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <opencv/cv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

int main() {
    std::string img_path = "../data/image.png"; // Path to the image file
    cv::Mat img = cv::imread(img_path, cv::IMREAD_GRAYSCALE); // Read the image in grayscale

    // the original size is 4000x3000, maybe too large for some systems, resize it to 1000x750
    // if (img.cols > 1000 || img.rows > 750) {
    //     cv::resize(img, img, cv::Size(1000, 750), cv::INTER_AREA);
    // }

    if (img.empty()) {
        std::cerr << "Error: Failed to load image from " << img_path << std::endl;
        return -1;
    }

    if (img.cols <= 0 || img.rows <= 0) {
        std::cerr << "Error: Invalid image dimensions." << std::endl;
        return -1;
    }

    // random sample a start and goal point that are not in the obstacle area
    cv::Point start, goal;
    bool found_start = false, found_goal = false;
    while (!found_start) {
        int x = rand() % img.cols;
        int y = rand() % img.rows;
        if (img.at<uchar>(y, x) < 128 && !found_start) { // Check if the pixel is not an obstacle
            start = cv::Point(x, y);
            found_start = true;
        }
    }
    while (!found_goal) {
        int x = rand() % img.cols;
        int y = rand() % img.rows;
        // check closeness to start point
        if (std::abs(x - start.x) < 1000 && std::abs(y - start.y) < 1000) {
            continue; // Skip if the goal is too close
        }
        if (img.at<uchar>(y, x) < 128 && !found_goal) { // Check if the pixel is not an obstacle
            goal = cv::Point(x, y);
            found_goal = true;
        }
    }


    std::vector<double> origin{0.0, 0.0, 0.0}; // Set origin at (0, 0, 0)
    double resolution = 0.1;
    std::vector<double> start_vec{static_cast<double>(start.x * resolution), static_cast<double>(start.y * resolution), 0.0};
    std::vector<double> goal_vec{static_cast<double>(goal.x * resolution), static_cast<double>(goal.y * resolution), 0.0};

    std::vector<int> dim{img.cols, img.rows, 1}; // Set the number of cells in each dimension
    std::vector<int> data; // occupancy data, subscript follows: id = x + dim.x * y + dim.x * dim.y * z;
    data.resize(dim[0] * dim[1] * dim[2], 0); // Initialize as free map, free cell has 0 occupancy

    // Fill the occupancy data based on the image
    for (int y = 0; y < img.rows; ++y) {
        for (int x = 0; x < img.cols; ++x) {
            int id = x + dim[0] * y; // Calculate the index in the data vector
            if (img.at<uchar>(y, x) > 200) { // If the pixel is dark (obstacle)
                data[id] = 100; // Mark as occupied
            } else {
                data[id] = 0; // Mark as free
            }
        }
    }

    std::cout << "dim: " << dim[0] << " x " << dim[1] << " x " << dim[2] << std::endl;

    YAML::Emitter out;
    out << YAML::BeginSeq;
    // Encode start coordinate
    out << YAML::BeginMap;
    out << YAML::Key << "start" << YAML::Value << YAML::Flow << start_vec;
    out << YAML::EndMap;
    // Encode goal coordinate
    out << YAML::BeginMap;
    out << YAML::Key << "goal" << YAML::Value << YAML::Flow << goal_vec;
    out << YAML::EndMap;
    // Encode origin coordinate
    out << YAML::BeginMap;
    out << YAML::Key << "origin" << YAML::Value << YAML::Flow << origin;
    out << YAML::EndMap;
    // Encode dimension as number of cells
    out << YAML::BeginMap;
    out << YAML::Key << "dim" << YAML::Value << YAML::Flow << dim;
    out << YAML::EndMap;
    // Encode resolution
    out << YAML::BeginMap;
    out << YAML::Key << "resolution" << YAML::Value << resolution;
    out << YAML::EndMap;
    // Encode occupancy
    out << YAML::BeginMap;
    out << YAML::Key << "data" << YAML::Value << YAML::Flow << data;
    out << YAML::EndMap;

    out << YAML::EndSeq;
    // std::cout << "Here is the example map:\n" << out.c_str() << std::endl;

    std::ofstream file;
    file.open("image.yaml");
    file << out.c_str();
    file.close();

    return 0;
}
