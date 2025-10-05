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

    double cost;

    Solution() : cost(0.0) {}
    double computeCost(Eigen::Vector3d bounds); // returns cost

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
    const int numOfWaypoints = 30;
    Eigen::Vector3d bounds;
    int archiveSize;
    Eigen::Vector3d START;
    Eigen::Vector3d GOAL;

    Archive *archive;
    std::vector<double> weights;
    std::vector<double> probs;
    std::vector<std::vector<int>> heightmap;

    ACOSRAR(Eigen::Vector3d s, Eigen::Vector3d g, Eigen::Vector3d b, int aSize, std::vector<std::vector<int>> h);

    void cycle();

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