#include "sim/sim.hpp"
#include "algo.hpp"
#include <cmath>

using namespace std;

class Node {
public:
    Eigen::Vector3d pos;
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
        H = 1 * (goalPos - pos).norm();
        F = G + H;

        for (const auto& t : threats) {
            double dxz = (pos - t.getCenter()).head<2>().norm();
            bool withinRadius = dxz <= t.getR() + 15;
            bool withinHeight = abs(pos.z() - t.getCenter().z()) <= t.getH() / 2.0;
            if (withinRadius && withinHeight) {
                F = G = H = numeric_limits<float>::infinity();
                return;
            }
        }

        int x = (int)pos.x();
        int y = (int)pos.y();


        if (pos.x() < 0 || pos.y() < 0 || pos.z() < 0 ||
            pos.x() > bounds.x() || pos.y() > bounds.y() || pos.z() > bounds.z()) {
            F = G = H = numeric_limits<float>::infinity();
            return;
        } 
        
        if (pos.z() < heightmap[y][x] + 20) {
            F = G = H = numeric_limits<float>::infinity();
            return;
        }


    }

};

vector<Eigen::Vector3d> SAS(const Eigen::Vector3d &startPos, const Eigen::Vector3d &goalPos, const Eigen::Vector3d &bounds, const vector<Threat> threats, const vector<vector<int>> &heightmap) {
    const float maxTheta = M_PI / 3;
    const float minLength = 200;

    vector<Node> toSearch;
    vector<Node> processed;

    Node start;
    start.pos = startPos;
    start.G = 0;
    start.init(startPos, goalPos, bounds, threats, heightmap);
    start.F = start.G + start.H;

    if (start.F == numeric_limits<float>::infinity()) {
        cout << "Start node inside threat.\n";
        exit(1);
    }

    toSearch.push_back(start);
    Node* finalNode = nullptr;

    while (!toSearch.empty()) {
        auto currentIt = min_element(toSearch.begin(), toSearch.end(), [] (const Node &a, const Node &b) {
            return (a.F < b.F) || (a.F == b.F && a.H < b.H);
        });

        Node current = *currentIt;

        toSearch.erase(currentIt);
        processed.push_back(current);

        cout << "Goal dist: " << (current.pos - goalPos).norm() << "\n";
        if ((current.pos - goalPos).norm() < 50.0) {
            finalNode = &processed.back();
            break;
        }

        vector<Eigen::Vector3d> viablePaths;

        // Eigen::Vector3d axis = (goalPos - current.pos).normalized();
        // Eigen::Vector3d ref  = axis.unitOrthogonal().normalized();
        // Eigen::Vector3d up   = axis.cross(ref).normalized();

        // double step = maxTheta / 5.0;

        // for (double theta = -maxTheta / 2.0; theta <= maxTheta / 2.0; theta += step) {
        //     for (double phi = -maxTheta / 2.0; phi <= maxTheta / 2.0; phi += step) {
        //         Eigen::Vector3d dir = (ref * cos(theta) * cos(phi) + up  * sin(theta) * cos(phi) + axis * sin(phi)).normalized();

        //         Eigen::Vector3d newPos = current.pos + dir.normalized() * minLength;
        //         viablePaths.push_back(newPos);

        //         // std::cout << "Neighbor: "
        //         //         << newPos.x() << " "
        //         //         << newPos.y() << " "
        //         //         << newPos.z() << "\n";
        //     }
        // }
        // Eigen::Vector3d axis = (goalPos - current.pos).normalized();
        // Eigen::Vector3d ref  = axis.unitOrthogonal().normalized();
        // Eigen::Vector3d up   = axis.cross(ref).normalized();

        // for(double theta = -maxTheta/2; theta <= maxTheta/2; theta += step){
        //     for(double phi = -maxTheta/2; phi <= maxTheta/2; phi += step){
        //         Eigen::Vector3d dir = axis;
        //         dir = Eigen::AngleAxisd(theta, up).toRotationMatrix() * dir;
        //         dir = Eigen::AngleAxisd(phi, ref).toRotationMatrix() * dir;
        //         Eigen::Vector3d newPos = current.pos + dir * minLength;
        //         viablePaths.push_back(newPos);
        //     }
        // }
        Eigen::Vector3d axis = (goalPos - current.pos).normalized();
        Eigen::Vector3d ref  = axis.unitOrthogonal().normalized();
        Eigen::Vector3d up   = axis.cross(ref).normalized();

        double maxDeviation = maxTheta / 4.0;
        double coneStep = maxDeviation / 2.0;

        for(double theta = -maxDeviation; theta <= maxDeviation; theta += coneStep){
            for(double phi = -maxDeviation; phi <= maxDeviation; phi += coneStep){
                Eigen::Vector3d dir = axis;
                dir = Eigen::AngleAxisd(theta, up) * dir;
                dir = Eigen::AngleAxisd(phi, ref) * dir;
                Eigen::Vector3d newPos = current.pos + dir * minLength;
                viablePaths.push_back(newPos);
            }
        }

        // cout << "Neighbor cycle done \n";


        for (Eigen::Vector3d vPath : viablePaths) {
            Node path;
            path.pos = vPath;
            path.G = current.G + (current.pos - path.pos).norm();
            path.init(startPos, goalPos, bounds, threats, heightmap);
            // cout << "initialized path at: " << path.pos.x() << " " << path.pos.y() << " " << path.pos.z() << "\n";
            if (!std::isfinite(path.F)) {
                // cout << "skipping path due to F being inf." << path.F << ", G: " << path.G << ", H: " << path.H << "\n"; 
                continue;
            } 

            bool inProcessed = any_of(processed.begin(), processed.end(),
                [&](const Node &p) {return p.pos.isApprox(path.pos,  1e-6);});
            if(inProcessed) {
                // cout << "skipping path due to being in processed\n"; 
                continue;
            }
        
            float tentativeG = current.G + (current.pos - path.pos).norm();
            auto it = find_if(toSearch.begin(), toSearch.end(),
                [&](const Node &t) {return t.pos.isApprox(path.pos, 1e-6); });

            cout << "Processing: " << path.pos.x() << " " << path.pos.y() << " " << path.pos.z() << "\n"; 

            if (it == toSearch.end()) {
                // cout << "adding this neightbor to search list\n";
                path.G = tentativeG;
                path.F = path.G + path.H;
                path.connection = &processed.back();
                toSearch.push_back(path);
            } else if (tentativeG < it->G) {
                it->G = tentativeG;
                it->F = it->G + it->H;
                it->connection = &processed.back();
            }
            
        }
 
    }

    vector<Eigen::Vector3d> path;
    Node* node = finalNode;
    cout << "nodes\n";
    while (node) {
        cout << node->pos.x() << " " << node->pos.y() << " " << node->pos.z() << "\n";
        if (node->pos.x() == 0 && node->pos.y() == 0 && node->pos.z() == 0)
            break;
        path.push_back(node->pos);
        node = node->connection;
    }

    path.push_back(startPos);
    reverse(path.begin(), path.end());
    path.push_back(goalPos);
    return path;
}



// fills the waypoints/paths in map
void Map::pathfind() {
    std::cout << "\ncalling pathfind\n";

    // pass acosrar bounds
    path = SAS(startPos, goals[0], bounds, threats, heightmap);

    // path = std::move(algo.archive->solutions[0].waypoints);

    std::cout << "\ncalled pathfind\n";
}