#include "sim/sim.hpp"
#include "algo.hpp"

using namespace std;

class Node {
public:
    Eigen::Vector3d coords;
    float G = numeric_limits<float>::infinity();
    float H = numeric_limits<float>::infinity();
    float F = numeric_limits<float>::infinity();
    Node* connection = nullptr;

    void init(const Eigen::Vector3d& startPos,
              const Eigen::Vector3d& goalPos,
              const Eigen::Vector3d& bounds,
              const vector<Threat>& threats,
              const vector<vector<int>>& heightmap) 
    {
        H = 2.5 * (goalPos - coords).norm();
        // G will be updated later when computing path cost
        F = G + H;

        // Check threats
        for (const auto& t : threats) {
            double dxz = (coords - t.getCenter()).head<2>().norm();
            bool withinRadius = dxz <= t.getR() + 15;
            bool withinHeight = abs(coords.z() - t.getCenter().z()) <= t.getH() / 2.0;
            if (withinRadius && withinHeight) {
                cout << "inside threat\n";
                F = G = H = numeric_limits<float>::infinity();
            }
        }

        // Check heightmap
        int x = (int)coords.x();
        int y = (int)coords.y();
        if (x < 0 || y < 0 || y >= heightmap.size() || x >= heightmap[0].size() || coords.z() < heightmap[y][x] + 20) {
            // cout << "height issue\n";
            F = G = H = numeric_limits<float>::infinity();
        }

        // Check bounds
        if (coords.x() < 0 || coords.y() < 0 || coords.z() < 0 ||
            coords.x() > bounds.x() || coords.y() > bounds.y() || coords.z() > bounds.z()) {
            // cout << "bounds issue\n";
            F = G = H = numeric_limits<float>::infinity();
        }
    }
};

vector<Eigen::Vector3d> Astar(const Eigen::Vector3d& startPos, const Eigen::Vector3d& goalPos, const Eigen::Vector3d& bounds, const vector<Threat>& threats, const vector<vector<int>>& heightmap) { 
    cout << "entering\n";
    vector<Node> toSearch;
    vector<Node> processed;

    Node startNode;
    startNode.coords = startPos;
    startNode.G = 0;
    startNode.init(startPos, goalPos, bounds, threats, heightmap);
    startNode.F = startNode.G + startNode.H;

    toSearch.push_back(startNode);

    Node* finalGoal = nullptr;

    while (!toSearch.empty()) {
        // Find node with lowest F (tie-breaker H)
        auto currentIt = min_element(toSearch.begin(), toSearch.end(),
                                     [](const Node& a, const Node& b) {
                                         return (a.F < b.F) || (a.F == b.F && a.H < b.H);
                                     });
        Node current = *currentIt;
        // cout << "Processing: " << current.coords.transpose() << "\n";


        toSearch.erase(currentIt);
        processed.push_back(current);

        // Check if reached goal
        cout << "Goal dist: " << (current.coords - goalPos).norm() << "\n";
        if ((current.coords - goalPos).norm() < 25.0) {
            finalGoal = &processed.back();
            break;
        }

        // Generate neighbors (8 directions in XY plane, keep Z same)
        vector<Eigen::Vector3d> neighborCoords;

        double step = 10.0;

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    neighborCoords.push_back(current.coords + step * Eigen::Vector3d(dx, dy, dz));
                }
            }
        }

        for (auto& nCoord : neighborCoords) {
            Node neighbor;
            neighbor.G = current.G;
            neighbor.coords = nCoord;
            neighbor.init(startPos, goalPos, bounds, threats, heightmap);

            // Skip if in processed
            bool inProcessed = any_of(processed.begin(), processed.end(),
                                      [&](const Node& p){ return p.coords.isApprox(neighbor.coords, 1e-6); });
            if (inProcessed)
                continue;

            float tentativeG = current.G + (current.coords - neighbor.coords).norm();
            // Check if neighbor is in toSearch
            auto it = find_if(toSearch.begin(), toSearch.end(),
                              [&](const Node& t){ return t.coords.isApprox(neighbor.coords, 1e-6); });

            if (it == toSearch.end()) {
                neighbor.G = tentativeG;
                neighbor.F = neighbor.G + neighbor.H;
                neighbor.connection = &processed.back();
                toSearch.push_back(neighbor);
            } else if (tentativeG < it->G) {
                it->G = tentativeG;
                it->F = it->G + it->H;
                it->connection = &processed.back();
            }
        }
    }

    cout << "exit loop\n";;
    // Reconstruct path
    vector<Eigen::Vector3d> path;
    Node* node = finalGoal;
    while (node) {
        if (node->coords.x() < 1 || node->coords.y() < 1 || node->coords.z() < 1)
            break;
        cout << node->coords.x() << " " << node->coords.y() << " " << node->coords.z() << "\n";
        path.push_back(node->coords);
        node = node->connection;
    }
    path.push_back(startPos);
    reverse(path.begin(), path.end());
    // path.push_back(goalPos);
    return path;
}

// fills the waypoints/paths in map
void Map::pathfind() {
    std::cout << "\ncalling pathfind\n";

    path = Astar(startPos, goals[0], bounds, threats, heightmap);

    // cout << "Path:\n";
    // for (auto& p : path) {
    //     cout << p.transpose() << "\n";
    // }

    std::cout << "\ncalled pathfind\n";
}