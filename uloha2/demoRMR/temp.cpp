#include <vector>
#include <iostream>
#include <cmath>

// Define the grid map parameters
const int gridWidth = 100;   // Number of grid cells in the x direction
const int gridHeight = 100;  // Number of grid cells in the y direction
const float cellSize = 0.1;  // Size of each grid cell in meters

// Define the clustering parameters
const float minClusterSize = 0.5;   // Minimum size of a cluster in meters
const float maxClusterSize = 2.0;   // Maximum size of a cluster in meters
const float clusterTolerance = 0.2; // Tolerance distance for clustering in meters

// Convert polar coordinates to Cartesian coordinates
void polarToCartesian(float distance, float angle, float& x, float& y) {
    x = distance * std::sin(angle);
    y = distance * std::cos(angle);
}

// Cluster the LiDAR points into obstacles
void clusterLiDAR(const std::vector<float>& distances, const std::vector<float>& angles) {
    // Initialize the grid map
    std::vector<std::vector<int>> gridMap(gridHeight, std::vector<int>(gridWidth, 0));

    // Convert each LiDAR point to Cartesian coordinates and mark its grid cell as occupied
    for (int i = 0; i < distances.size(); ++i) {
        float x, y;
        polarToCartesian(distances[i], angles[i], x, y);
        int gridX = std::floor((x + gridWidth * cellSize / 2) / cellSize);
        int gridY = std::floor((y + gridHeight * cellSize / 2) / cellSize);
        if (gridX >= 0 && gridX < gridWidth && gridY >= 0 && gridY < gridHeight) {
            gridMap[gridY][gridX] = 1;
        }
    }

    // Cluster the occupied grid cells into obstacles
    for (int y = 0; y < gridHeight; ++y) {
        for (int x = 0; x < gridWidth; ++x) {
            if (gridMap[y][x] == 1) {
                // Find nearby occupied grid cells using a sliding window
                std::vector<int> nearbyX, nearbyY;
                for (int j = std::max(0, y - 1); j <= std::min(gridHeight - 1, y + 1); ++j) {
                    for (int i = std::max(0, x - 1); i <= std::min(gridWidth - 1, x + 1); ++i) {
                        if (gridMap[j][i] == 1) {
                            nearbyX.push_back(i);
                            nearbyY.push_back(j);
                        }
                    }
                }
                // If there are enough nearby occupied grid cells, cluster them
}
