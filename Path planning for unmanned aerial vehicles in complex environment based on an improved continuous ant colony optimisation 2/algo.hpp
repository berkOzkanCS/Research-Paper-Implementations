#ifndef ALGO_HPP
#define ALGO_HPP

#include <Eigen/Dense>
#include <memory>
#include <random>
#include <string>
#include <fstream>
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <iostream>

class Solution {
    public:
    std::vector<Eigen::Vector3d> waypoints;
    double D = 10.0, L = 5.0;
    double cost;
    double bd = 1.5;
    double bc = 1.0;
    double be = 0.2;
    double bcorn = 1.0;

    Solution() : cost(0.0) {}
    double computeCost(Eigen::Vector3d bounds, std::vector<Threat> threats, std::vector<std::vector<int>> heightmap);
    double computeDistanceCost(Eigen::Vector3d bounds);
    double computeCollisionCost(Eigen::Vector3d bounds, std::vector<Threat> threats);
    double computeElevationCost(Eigen::Vector3d bounds, std::vector<std::vector<int>> heightmap);
    double computeCornerCost();
    void repairWaypoint(char domain, int waypointIndex, Eigen::Vector3d bounds, std::vector<Threat> threats, std::vector<std::vector<int>> heightmap);
};

class Archive {
    public:
    std::vector<Solution> solutions;

    void sortByCost();
};

class ACOSRAR {
    public:
    double l = 0.01; // [0.1,0.5]
    double ksi = 1.0; // [0.1,1.0]
    const double mu = 0.8; // [0.001, 1.0] small = slower to explore, large = faster to explore
    const double alpha = 1.0; // [0,2] small = large jumps, large = smal lumps

    const int numOfWaypoints = 10;
    Eigen::Vector3d bounds;
    int archiveSize;
    Eigen::Vector3d START;
    Eigen::Vector3d GOAL;

    Archive *archive;
    std::vector<double> weights;
    std::vector<double> probs;
    std::vector<std::vector<int>> heightmap;

    ACOSRAR(Eigen::Vector3d s, Eigen::Vector3d g, Eigen::Vector3d b, int aSize, std::vector<std::vector<int>> h);

    void cycle(int curIter, int maxIter, std::vector<Threat> threats);

    Solution createNewSolution(Solution &guide, int guideIndex);
    void createInitialArchive();

    ~ACOSRAR() {
        delete archive; 
    }
};  

std::random_device rd;
std::mt19937 gen(rd());

double gaussian(double mean, double stddev) {
    std::normal_distribution<> dis(mean, stddev);
    return dis(gen);
}


#endif