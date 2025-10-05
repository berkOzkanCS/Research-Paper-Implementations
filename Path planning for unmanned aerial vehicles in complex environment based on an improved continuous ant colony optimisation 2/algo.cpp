#include "sim/sim.hpp"
#include "algo.hpp"



double Solution::computeCost(Eigen::Vector3d bounds) {
    double distCost = 0.0;
    for (size_t i = 0; i < waypoints.size() - 1; i++) {
    if (waypoints[i].x() < 0 || waypoints[i].x() > bounds.x() ||
        waypoints[i].y() < 0 || waypoints[i].y() > bounds.y() ||
        waypoints[i].z() < 0 || waypoints[i].z() > bounds.z()) {
        return std::numeric_limits<double>::infinity();
    }
        distCost += (waypoints[i+1]-waypoints[i]).norm();
    }
    return distCost;
}

void Archive::sortByCost() {
    std::sort(solutions.begin(), solutions.end(), [] (const Solution& a, const Solution& b) {
        return a.cost < b.cost;
    });
}

void ACOSRAR::cycle() {
    // Step 1: sort archive according to cost
    archive->sortByCost();

    // Step 2 & 3 done in the constructor

    // Step 4: create X new paths to replace the worst X from the archive
        // Step 4.1: pick a guiding solution based off the probabilities
            // create CDF
            std::vector<double> CDF;
            CDF.push_back(probs[0]);
            for (size_t i = 1; i < probs.size(); i++) {
                CDF.push_back(CDF[i-1] + probs[i]);
            }
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0.0, 1.0);
            double r = dis(gen);
            auto it = std::lower_bound(CDF.begin(), CDF.end(), r);

        int guidingIndex = std::distance(CDF.begin(), it);

        // Step 4.2: generate k solutions
        std::vector<Solution> newSolutions;
        
        for (size_t i = 0; i < archiveSize; i++) {
            Solution s = createNewSolution(archive->solutions[guidingIndex], guidingIndex);
            s.cost = s.computeCost(bounds);
            newSolutions.push_back(s);
        }
        // replace old solutions
        archive->solutions = std::move(newSolutions);
}

Solution ACOSRAR::createNewSolution(Solution &guide, int guideIndex) {
    Solution s;
    s.waypoints.push_back(START);
    for (size_t i = 1; i < guide.waypoints.size() - 1; i++) { //first and last contain start and goal
        // calculate the std for each dim
        double sumX = 0.0;
        double sumY = 0.0;
        double sumZ = 0.0;
        for (size_t j = 0; j < archiveSize; j++) {
            if (guideIndex == j)
                continue;
            sumX += abs(archive->solutions[j].waypoints[i].x() - guide.waypoints[i].x()) / (archiveSize - 1);
            sumY += abs(archive->solutions[j].waypoints[i].y() - guide.waypoints[i].y()) / (archiveSize - 1);
            sumZ += abs(archive->solutions[j].waypoints[i].z() - guide.waypoints[i].z()) / (archiveSize - 1);
        }

        double x_std = ksi * sumX;
        double y_std = ksi * sumY;
        double z_std = ksi * sumZ;

        double x_mod = abs(gaussian(guide.waypoints[i].x(), x_std));
        x_mod = (x_mod > bounds.x()) ? bounds.x() : x_mod;
        double y_mod = abs(gaussian(guide.waypoints[i].y(), y_std));
        y_mod = (y_mod > bounds.y()) ? bounds.y() : y_mod;
        double z_mod = abs(gaussian(guide.waypoints[i].z(), z_std));
        z_mod = (z_mod > bounds.z()) ? bounds.z() : z_mod;
        // std::cout << x_mod << ", " << y_mod << ", " << z_mod<< "\n";
        s.waypoints.push_back(Eigen::Vector3d(x_mod, y_mod, z_mod));
    }
    s.waypoints.push_back(GOAL);
    return s;
}

void ACOSRAR::createInitialArchive() {
    static std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> locX(0, bounds.x());
    std::uniform_int_distribution<int> locY(0, bounds.y());
    std::uniform_int_distribution<int> locZ(0, bounds.z());
    archive->solutions.clear();
    for (size_t i = 0; i < archiveSize; i++) {
        Solution s;
        s.waypoints.push_back(Eigen::Vector3d(START.x(), START.y(), START.z()));

        for (size_t j = 1; j < numOfWaypoints - 1; j++) {
            s.waypoints.push_back(Eigen::Vector3d(locX(rng), locY(rng), locZ(rng)));
        }

        s.waypoints.push_back(Eigen::Vector3d(GOAL.x(), GOAL.y(), GOAL.z()));

        s.cost = s.computeCost(bounds);

        archive->solutions.push_back(s);
    }
}

ACOSRAR::ACOSRAR(Eigen::Vector3d s, Eigen::Vector3d g, Eigen::Vector3d b, int aSize, std::vector<std::vector<int>> h) {
    START = s;
    GOAL = g;
    bounds = b;
    heightmap = h;
    archiveSize = aSize;
    double totalW = 0.0;
    const int k = archiveSize;
    archive = new Archive();
    // Step 2: create weights
    for (size_t i = 0; i < k; i++) {
        double wi = 1.0 / (l * k * sqrt(2*M_PI)) * exp(-1.0 * pow((i - 1.0),2) / (2.0 * l*l * k*k));
        weights.push_back(wi);
        totalW += wi;
    }

    // Step 3: determine probabilities
    for (size_t i = 0; i < k; i++) {
        double psi = weights[i] / totalW;
        probs.push_back(psi);
    }
}



// fills the waypoints/paths in map
void Map::pathfind() {

    // pass acosrar bounds
    const int archiveSize = 200;
    const int numOfCycles = 1000;
    ACOSRAR algo(startPos, goals[0], bounds, archiveSize, heightmap);
    algo.createInitialArchive();

    // for (size_t i = 0; i < numOfCycles; i++) {
    //     algo.cycle();
    //     algo.archive->sortByCost();
    //     std::cout << "Best Cost: " << algo.archive->solutions[0].cost << "\n";

    //     if (i % 10 == 0) {
    //         int width = 50;
    //         int pos = width * i / numOfCycles;
    //         std::cout << "\r[";
    //         for (int j = 0; j < width; ++j)
    //             std::cout << (j < pos ? "=" : (j == pos ? ">" : " "));
    //         std::cout << "] " << i * 100 / numOfCycles << "%";
    //         std::cout.flush();
    //     }
    // }

    double lastbest = -1.0;
    do {
        algo.cycle();
        algo.archive->sortByCost();
        std::cout << "Best Cost: " << algo.archive->solutions[0].cost << "\n";
        if (lastbest == algo.archive->solutions[0].cost) {
            algo.archive->solutions[0] = algo.createNewSolution(algo.archive->solutions[0], archiveSize-1);
        }
        lastbest = algo.archive->solutions[0].cost;
    } while (algo.archive->solutions[0].cost > 750);

    algo.archive->sortByCost();

    path = std::move(algo.archive->solutions[0].waypoints);

    std::cout << "calling pathfind\n";
}