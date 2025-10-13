#include "algo.hpp"


vector<Obstacle> obstacles;
vector<Node*> pathNodes;
Node startNode;
Node goalNode;
vector<Node*> openSet;
vector<Node*> closedSet;

void init_game(Vector2d &s, Vector2d &g) {
    startNode.pos = s;
    startNode.G = 0;
    startNode.H = (g - s).norm();
    startNode.F = startNode.G + startNode.H;
    goalNode.pos = g;

    openSet.push_back(&startNode);

    // generate obstacles
    std::random_device dev;
    std::mt19937 rng(dev());
    uniform_int_distribution<std::mt19937::result_type> distR((MAX_X + MAX_Y)/2*0.05, (MAX_X + MAX_Y)/2*0.1);
    uniform_int_distribution<std::mt19937::result_type> distCenterX(MAX_X*0.1, MAX_X*0.9);
    uniform_int_distribution<std::mt19937::result_type> distCenterY(MAX_Y*0.4, MAX_Y*0.6);

    for (size_t i = 0; i < OBSTACLE_CNT; i++) {
        int radius;
        Vector2d center;
        
        do {
            radius = distR(rng);
            center = Vector2d(distCenterX(rng), distCenterY(rng));
        } while (inObstacle(startNode.pos, Obstacle(center, radius)));
        
        if (radius > 0 && radius < 100 && center.x() > 0 && center.x() < MAX_X && center.y() > 0 && center.y() < MAX_Y) {
            obstacles.push_back(Obstacle(center, radius));
        }
    }

}

vector<Node*> generateAstarPath() {
    while(!openSet.empty()) {
        // auto comp = [](const Node& a, const Node& b) {
        //             return (a.F < b.F) || (a.F == b.F && a.H < b.H); 
        //         };
        auto currentIt = std::min_element(openSet.begin(), openSet.end(), NodeComparator);

        Node* current = *currentIt;

        double goalThreshold = 20.0; 

        if ((current->pos - goalNode.pos).norm() <= goalThreshold) {
            while (current->connection != nullptr) {
                pathNodes.push_back(current);
                current = current->connection;
            }
            pathNodes.push_back(&startNode);
            std::reverse(pathNodes.begin(), pathNodes.end());
            pathNodes.push_back(&goalNode);
            return pathNodes;
        }

        closedSet.push_back(current);
        openSet.erase(currentIt);

        // generate neighbors
        /**
         * if start then look at all 8 nodes besides it
         * if not start then look at the three in front and get direction based off previous node
        */
       vector<Node*> neighbor;
        // first node 
        if (current->connection == nullptr) {
            for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1; j++) {
                    if (i != 0 && j != 0) {
                        Node* n = new Node();
                        n->pos = Vector2d(current->pos.x()+i*STEP_LENGTH, current->pos.y()+j*STEP_LENGTH);

                        // checking if node is viable
                        if (n->pos.x() < 0 || n->pos.x() > MAX_X || n->pos.y() < 0 || n->pos.y() > MAX_Y) {
                            delete n;
                            continue;
                        }

                        bool collision = false;
                        for (Obstacle o : obstacles) {
                            if (inObstacle(n->pos, o)) {
                                collision = true;
                                break;
                            }
                        }

                        if (collision) { delete n; continue; }
                        
                        n->G = current->G + (current->pos - n->pos).norm();
                        n->H = (goalNode.pos - n->pos).norm();
                        n->F = n->G + n->H;
                        n->connection = current;
                        neighbor.push_back(n);
                    }
                }
            } 
        } else {
            Vector2d dir = (current->pos - current->connection->pos).normalized();
            Vector2d nextPixel = (current->pos + dir).array().round();

            Vector2d side1, side2;

            if (dir.x() > 0 && dir.y() > 0) { 
                side1 = nextPixel + Vector2d(-1*STEP_LENGTH, 0);
                side2 = nextPixel + Vector2d(0, -1*STEP_LENGTH);
            } else if (dir.x() == 0) { 
                side1 = nextPixel + Vector2d(1*STEP_LENGTH, 0);
                side2 = nextPixel + Vector2d(-1*STEP_LENGTH, 0);
            } else if (dir.y() == 0) { 
                side1 = nextPixel + Vector2d(0, 1*STEP_LENGTH);
                side2 = nextPixel + Vector2d(0,-1*STEP_LENGTH);
            } else if (dir.x() > 0 && dir.y() < 0) {
                side1 = nextPixel + Vector2d(-1*STEP_LENGTH, 0);
                side2 = nextPixel + Vector2d(0, 1*STEP_LENGTH);
            } else if (dir.x() < 0 && dir.y() < 0) {
                side1 = nextPixel + Vector2d(1*STEP_LENGTH, 0);
                side2 = nextPixel + Vector2d(0, 1*STEP_LENGTH);
            } else if (dir.x() < 0 && dir.y() > 0) {
                side1 = nextPixel + Vector2d(1*STEP_LENGTH, 0);
                side2 = nextPixel + Vector2d(0, -1*STEP_LENGTH);
            }

            std::vector<Eigen::Vector2d> positions;
            positions.push_back(nextPixel);
            positions.push_back(side1);
            positions.push_back(side2);

            for (const auto& pos : positions) {
                if (isBlocked(pos, MAX_X, MAX_Y, obstacles)) continue;
                Node* n = new Node();
                n->pos = pos;
                n->G = current->G + (current->pos - n->pos).norm();
                n->H = (goalNode.pos - n->pos).norm();
                n->F = n->G + n->H;
                n->connection = current;
                neighbor.push_back(n);
            }
        }

        for (Node* n : neighbor) {
            if (isInClosedSet(closedSet, n)) {
                delete n;
                continue;
            }

            Node* existing = findInOpenSet(openSet, n);
            double tentative_G = current->G + (current->pos - n->pos).norm();

            if (existing == nullptr) {
                n->G = tentative_G;
                n->H = (goalNode.pos - n->pos).norm();
                n->F = n->G + n->H;
                n->connection = current;
                openSet.push_back(n);
            } else {
                if (tentative_G < existing->G) {
                    existing->G = tentative_G;
                    existing->F = existing->G + existing->H;
                    existing->connection = current;
                }
                delete n;
            }
        }

        // openSet.insert(openSet.end(), std::make_move_iterator(neighbor.begin()), std::make_move_iterator(neighbor.end()));

    }
    return pathNodes;
}

void savePathAndObstacles(const vector<Node*> pathNodes, const std::string& filename) {
    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        std::cerr << "Failed to open file " << filename << "\n";
        return;
    }

    // Save map size
    outFile << MAX_X << " " << MAX_Y << "\n";

    // Save start and goal positions
    outFile << "START " << startNode.pos.x() << " " << startNode.pos.y() << "\n";
    outFile << "GOAL " << goalNode.pos.x() << " " << goalNode.pos.y() << "\n";

    // Save path nodes
    outFile << "PATH\n";
    for (const auto& node : pathNodes) {
        outFile << node->pos.x() << " " << node->pos.y() << "\n";
    }

    std::cout << "Saving " << obstacles.size() << " obstacles\n";
    // Save obstacles
    outFile << "OBSTACLES\n";
    for (const auto& o : obstacles) {
        outFile << o.c.x() << " " << o.c.y() << " " << o.r << "\n";
    }

    outFile.close();
    std::cout << "Path and obstacles saved to " << filename << "\n";
}



bool isInClosedSet(const std::vector<Node*>& closedSet, const Node* n) {
    for (Node* node : closedSet) {
        if (node->pos == n->pos) {
            return true;
        }
    }
    return false;
}

Node* findInOpenSet(const std::vector<Node*>& openSet, const Node* n) {
    for (Node* node : openSet) {
        if (node->pos == n->pos) {
            return node; // return existing node
        }
    }
    return nullptr; // not found
}

bool isBlocked(const Eigen::Vector2d& v,
               int MAX_X, int MAX_Y,
               const std::vector<Obstacle>& obstacles)
{
    // Out of bounds
    if (v.x() < 0 || v.x() > MAX_X || v.y() < 0 || v.y() > MAX_Y)
        return true;

    // Collision with any obstacle
    for (const auto& o : obstacles) {
        if (inObstacle(v, o))
            return true;
    }

    return false;
}

bool inObstacle(const Eigen::Vector2d& point, const Obstacle& o) {
    double dx = point.x() - o.c.x();
    double dy = point.y() - o.c.y();
    return (dx*dx + dy*dy) < (o.r * o.r);
}

bool NodeComparator(const Node* a, const Node* b) {
    return (a->F < b->F) || (a->F == b->F && a->H < b->H); 
}