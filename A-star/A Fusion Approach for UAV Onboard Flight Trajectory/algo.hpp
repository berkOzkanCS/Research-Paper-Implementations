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

const int MAX_X = 100;
const int MAX_Y = 100;
const int OBSTACLE_CNT = 4;
const int STEP_LENGTH = 1;
const int UAV_RADIUS = 5;

const double W1 = 0.4;
const double W2 = 0.3;
const double W3 = 0.3;
const double EPU = 5.9;
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
vector<Node*> updatePath();

void savePathAndObstacles(const vector<Node*> pathNodes, const std::string& filename = "path_data.txt");

bool smallestF(const Node* a, const Node* b);

bool isUnavailable(const Eigen::Vector2d& v);
Vector2d closestCirclePoint(const Vector2d &v, double &closestDist);

int numOfOccupiedCells(const Vector2d &p);
bool isInClosedSet(const vector<Node*>& closedSet, const Node* n);
Node* findInOpenSet(const vector<Node*>& openSet, const Node* n);


#endif