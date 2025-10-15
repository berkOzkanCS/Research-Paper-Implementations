#include "algo.hpp"

vector<Obstacle> obstacles;
vector<Node*> pathNodes;
Node startNode;
Node goalNode;
vector<Node*> openSet;
vector<Node*> closedSet;

void init_game(Vector2d &s, Vector2d &g) {
    startNode.pos = s;
    goalNode.pos = g;
    startNode.G = 0;
    startNode.H = (g-s).norm();
    startNode.F = startNode.G + startNode.H;

    openSet.push_back(&startNode);

    obstacles.push_back(Obstacle(Vector2d(40, 40), 10));
    obstacles.push_back(Obstacle(Vector2d(60, 40), 10));
    obstacles.push_back(Obstacle(Vector2d(65, 65), 10));
}

vector<Node*> generateAstarPath() {
    while(!openSet.empty()) {
        auto currentIt = std::min_element(openSet.begin(), openSet.end(), smallestF);
        Node* current = *currentIt;
        const int goalThresh = 10;
        if ((current->pos - goalNode.pos).norm() <= goalThresh) {
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

        vector<Node*> neighbour;
        Vector2d dir = (goalNode.pos - current->pos).normalized();

        if (current->connection != nullptr) {
            dir = (current->pos - current->connection->pos).normalized();
        } 
        
        vector<Vector2d> candidates;
        candidates.push_back(Vector2d( STEP_LENGTH,  0));
        candidates.push_back(Vector2d(-STEP_LENGTH,  0));
        candidates.push_back(Vector2d( 0,  STEP_LENGTH));
        candidates.push_back(Vector2d( 0, -STEP_LENGTH));
        candidates.push_back(Vector2d( STEP_LENGTH,  STEP_LENGTH));
        candidates.push_back(Vector2d(-STEP_LENGTH,  STEP_LENGTH));
        candidates.push_back(Vector2d( STEP_LENGTH, -STEP_LENGTH));
        candidates.push_back(Vector2d(-STEP_LENGTH, -STEP_LENGTH));

        Vector2d SE = (goalNode.pos - startNode.pos).normalized();  
        for (const auto& pos : candidates) {
            if (isUnavailable(current->pos + pos)) continue;

            Vector2d AE = (goalNode.pos - (current->pos + pos)).normalized();        
            double cosTheta = AE.dot(SE);                              

            if (cosTheta < 0.5) continue;

            Node* n = new Node();
            n->pos = current->pos + pos;
            n->G = current->G + (current->pos - n->pos).norm();

            //distancel from current to goal
            double dx = std::abs(goalNode.pos.x() - current->pos.x());
            double dy = std::abs(goalNode.pos.y() - current->pos.y());
            double h1 = dx + dy + (sqrt(3)-3)*std::min(dx, dy);
            double h2 = numOfOccupiedCells(n->pos);
            double closestDist = 0;
            auto p = closestCirclePoint(n->pos, closestDist);
            double h3 = (UAV_RADIUS + EPU) / (closestDist);

            n->H = W1*h1+W2*h2+W3*h3;   
            n->F = n->G + n->H;
            n->connection = current;
            neighbour.push_back(n);
        }

        for (Node* n : neighbour) {
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
    }
    return pathNodes;
}

vector<Node*> updatePath() {

    obstacles.push_back(Obstacle(Vector2d(40, 60), 10));

    if (pathNodes.size() < 2) return pathNodes;

    for (size_t i = 1; i < pathNodes.size() - 1; i++) {
        double closestDist = 0.0;
        Vector2d closestPoint = closestCirclePoint(pathNodes[i]->pos, closestDist);
        double range = (UAV_RADIUS + EPU) - closestDist;

        if (range > 0) {
            Vector2d dir = (pathNodes[i]->pos - closestPoint).normalized();
            Vector2d displacement = range * dir;
            pathNodes[i]->pos += displacement;
        }
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

bool isInClosedSet(const vector<Node*>& closedSet, const Node* n) {
    for (Node* node : closedSet) {
        if (node->pos == n->pos) {
            return true;
        }
    }
    return false;
}

Node* findInOpenSet(const vector<Node*>& openSet, const Node* n) {
    for (Node* node : openSet) {
        if (node->pos == n->pos) {
            return node; // return existing node
        }
    }
    return nullptr;
}

int numOfOccupiedCells(const Vector2d &p) {
    int occupiedCount = 0;
    double resolution = 0.1; // grid spacing
    for (double x = p.x() - UAV_RADIUS; x <= p.x() + UAV_RADIUS; x += resolution) {
        for (double y = p.y() - UAV_RADIUS; y <= p.y() + UAV_RADIUS; y += resolution) {
            Vector2d point(x, y);
            if ((point - p).norm() > UAV_RADIUS) continue; // outside circle of interest
            // check if inside any obstacle circle
            for (const auto& o : obstacles) {
                if ((point - o.c).norm() <= o.r) {
                    occupiedCount++;
                    break;
                }
            }
        }
    }

    return occupiedCount;
}

Vector2d closestCirclePoint(const Vector2d &v, double &closestDist) {
    Vector2d closestPoint;
    double minDist = std::numeric_limits<double>::max();

    for (const auto& o : obstacles) {
        Vector2d diff = v - o.c;
        double distToCenter = diff.norm();
        double distToBorder = distToCenter - o.r;
        if (distToBorder < minDist) {
            minDist = distToBorder;
            closestPoint = o.c + diff.normalized() * o.r;
        }
    }

    closestDist = minDist;
    return closestPoint;
}

bool isUnavailable(const Vector2d &v) {
    // Out of bounds
    if (v.x() < 0 || v.x() > MAX_X || v.y() < 0 || v.y() > MAX_Y)
        return true;

    // Collision with any obstacle
    for (const auto& o : obstacles) {
        double dx = v.x() - o.c.x();
        double dy = v.y() - o.c.y();
        if ((dx*dx + dy*dy) < (o.r * o.r) + UAV_RADIUS + 5)
            return true;
    }

    return false;
}

bool smallestF(const Node* a, const Node* b) {
    return (a->F < b->F) || (a->F == b->F && a->H < b->H); 
}