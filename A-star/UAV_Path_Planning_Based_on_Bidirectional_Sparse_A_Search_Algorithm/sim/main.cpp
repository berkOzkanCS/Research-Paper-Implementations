#include "sim.hpp"
#include <iostream>
#include <fstream>
#include <iostream>
#include <arpa/inet.h>
#include <sstream>
#include <string>



/**
 * usage: ./out "filepath.png" #obstacles" 
 */
int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: program <map_png> #obstacles\n";
        return 1;
    }
    std::string map_file_path = argv[1];
    std::ifstream map_file(map_file_path);
    if (!map_file) {
        std::cout << std::filesystem::current_path() << std::endl;
        std::cerr << "Failed to open " << map_file_path << "\n";
        return 1;
    }

    unsigned int width, height;

    map_file.seekg(16);
    map_file.read((char *)&width, 4);
    map_file.read((char *)&height, 4);

    const int MAP_WIDTH = ntohl(width);
    const int MAP_HEIGHT = ntohl(height);

    int obstaclesCnt = static_cast<int>(*argv[2]);
    Eigen::Vector3d start(450,150,50), goal(70, 250, 170);
    
    // automatically generates threats
    Map world(Eigen::Vector3d(MAP_WIDTH, MAP_HEIGHT, MAP_Z), start, {goal}, obstaclesCnt);
    world.readMap("heightmap2.bin", MAP_WIDTH, MAP_HEIGHT);
    // run algorithm using threats, start, goal, terrain
    world.pathfind();
    // world.pathGenerator(15, ' ');
    // world.printPath();

    world.saveToFile("map_data.txt");

    return 0;
}