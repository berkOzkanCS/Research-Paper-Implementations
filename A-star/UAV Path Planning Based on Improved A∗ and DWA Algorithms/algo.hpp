#ifndef SIM_HPP
#define SIM_HPP

#include "Eigen/Dense"
#include <random>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <initializer_list>

using namespace std;
using namespace Eigen;
// have obstancle coordiantes and radius
// have nodes used for travel

const int MAX_X = 100;
const int MAX_Y = 100;
const int OBSTACLE_CNT = 15;
const int STEP_LENGTH = 5;

struct Obstacle {
    Vector2d c;
    int r;

    Obstacle(const Eigen::Vector2d& center, int radius) : c(center), r(radius) {}
}; // circle

struct Node {
    Vector2d pos;
    double G, H, F;
    Node* connection = nullptr;
};

void init_game(Vector2d &s, Vector2d &g);
vector<Node*> generateAstarPath();
bool NodeComparator(const Node* a, const Node* b);

void savePathAndObstacles(const vector<Node*> pathNodes, const std::string& filename = "path_data.txt");

bool inObstacle(const Eigen::Vector2d& point, const Obstacle& o);
bool isBlocked(const Eigen::Vector2d& v, int MAX_X, int MAX_Y, const std::vector<Obstacle>& obstacles);
bool isInClosedSet(const std::vector<Node*>& closedSet, const Node* n);
Node* findInOpenSet(const std::vector<Node*>& openSet, const Node* n);
bool NodeComparator(const Node* a, const Node* b);


#endif