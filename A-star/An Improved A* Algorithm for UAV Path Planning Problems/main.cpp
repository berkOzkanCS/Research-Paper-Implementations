#include "algo.hpp"



int main() {

    Vector2d start(10,10);
    Vector2d goal(80,80);

    init_game(start, goal);    
    vector<Node*> path = generateAstarPath();
    savePathAndObstacles(path, "astar.txt");

    // REMEMBER TO DELETE new

    // generate a* path
    return 0;
}