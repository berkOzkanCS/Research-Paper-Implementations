#include "algo.hpp"

int main() {
    Vector2d s(10, 10);
    Vector2d g(80, 80);

    init_game(s, g);
    vector<Node*> path = generateAstarPath();
    savePathAndObstacles(path, "astar.txt");

    path = updatePath();
    savePathAndObstacles(path, "move.txt");

    for (Node* n : path) {
        if (n->connection != nullptr)
            delete n;
    }

    return 0;
}