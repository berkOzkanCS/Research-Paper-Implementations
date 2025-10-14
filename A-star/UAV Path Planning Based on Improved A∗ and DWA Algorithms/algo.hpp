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
vector<Node*> floyd(vector<Node*> p);
bool lineIntersectsAnyCircle(const Vector2d& p1, const Vector2d& p2, const vector<Obstacle>& obs);

vector<Vector2d> runDWA(const vector<Node*>& p);

void savePathAndObstacles(const vector<Node*> pathNodes, const std::string& filename = "path_data.txt");

bool inObstacle(const Eigen::Vector2d& point, const Obstacle& o);
bool isBlocked(const Eigen::Vector2d& v, int MAX_X, int MAX_Y, const std::vector<Obstacle>& obstacles);
bool isInClosedSet(const std::vector<Node*>& closedSet, const Node* n);
Node* findInOpenSet(const std::vector<Node*>& openSet, const Node* n);
bool NodeComparator(const Node* a, const Node* b);

struct DWAParams {
    double v_max = 5.0;
    double v_min = 0.0;
    double w_max = 1.0;
    double w_min = -1.0;
    double a_acc = 1.0;   // linear max accel
    double a_dec = 1.0;   // linear max decel (positive)
    double alpha_acc = 1.0; // angular accel
    double alpha_dec = 1.0; // angular decel (positive)
    double dt = 0.1;      // control timestep
    double predict_time = 1.0; // prediction horizon
    int v_samples = 7;
    int w_samples = 7;
    double safety_margin = 0.1; // extra clearance [m]
    // evaluation weights (tweak)
    double alpha_heading = 1.0;
    double beta0 = 1.0; // max speed weight
    double gamma_known = 1.0;
    double lambda_unknown = 1.0;
    double Ds = 2.0; // safety threshold for adaptive beta
    double eta = 0.5; // adaption strength
};

#endif