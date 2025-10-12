#ifndef SIM_HPP
#define SIM_HPP

#include <Eigen/Dense>
#include <memory>
#include <random>
#include <string>
#include <fstream>
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <iostream>

const int MAP_Z = 255;

class GeneratorStrategy {
    public:
        virtual void collisionCheck(Eigen::Vector3d v) = 0;
        virtual ~GeneratorStrategy() = default;
};

class CylinderStrategy : public GeneratorStrategy {
    public:
        void collisionCheck(Eigen::Vector3d v) override;
};

class DomeStrategy : public GeneratorStrategy {
    public:
        void collisionCheck(Eigen::Vector3d v) override;
};

class Threat {
    Eigen::Vector3d center;
    int R;
    int H;
    public:
        Threat(Eigen::Vector3d center, int R,int H);
        void setCenter(const Eigen::Vector3d& c);
        int getR() const { return R; }
        int getH() const { return H; }
        Eigen::Vector3d getCenter() const;
};

class Map {
    private:
        Eigen::Vector3d bounds;
        std::vector<Threat> threats;
        Eigen::Vector3d startPos;
        std::vector<Eigen::Vector3d> goals;

        std::vector<std::vector<int>> heightmap;

        std::vector<Eigen::Vector3d> path;

    public:
        Map();
        Map(Eigen::Vector3d b, std::vector<Threat> t, Eigen::Vector3d sP, std::vector<Eigen::Vector3d> g);
        Map(Eigen::Vector3d b, Eigen::Vector3d sP, std::vector<Eigen::Vector3d> g, int numberOfThreats);

        void setPath(std::vector<Eigen::Vector3d> p);

        std::vector<Eigen::Vector3d> pathGenerator(int numOfNodes, char setAxis);
        void printPath();


        void readMap(const std::string& filePath, int mapWidth, int mapHeight);
        void saveToFile(const std::string& filePath) const;

        void pathfind();
};

#endif