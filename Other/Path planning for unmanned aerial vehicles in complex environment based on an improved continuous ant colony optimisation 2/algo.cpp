#include "sim/sim.hpp"
#include "algo.hpp"


double Solution::computeCornerCost() {
    const double gamma1 = 0.7, gamma2 = 0.7; // [0.1,1.0]
    auto path = waypoints;
    if (path.size() < 3) return 0.0; // need at least 3 points to form a corner

    double cost = 0.0;
    std::vector<double> beta(path.size(), 0.0); // store vertical angles

    for (size_t j = 0; j < path.size() - 2; ++j) {
        Eigen::Vector3d v1 = path[j+1] - path[j];
        Eigen::Vector3d v2 = path[j+2] - path[j+1];

        // Horizontal projection onto XY plane
        Eigen::Vector2d v1_xy(v1.x(), v1.y());
        Eigen::Vector2d v2_xy(v2.x(), v2.y());

        // alpha: horizontal turning angle
        double cross = v1_xy.x()*v2_xy.y() - v1_xy.y()*v2_xy.x();
        double dot   = v1_xy.dot(v2_xy);
        double alpha = std::atan2(std::abs(cross), dot);

        // beta: vertical climbing angle
        double dz = v2.z();
        double beta_j = dz / v2.norm(); // climbing slope
        beta[j+1] = beta_j;

        double beta_diff = (j == 0) ? 0.0 : std::abs(beta[j+1] - beta[j]);

        cost += gamma1 * alpha + gamma2 * beta_diff;
    }

    return cost;
}

double lineSegmentCylinderDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Threat& cyl) {
    // Project cylinder center onto XY plane
    Eigen::Vector2d c_xy(cyl.getCenter().x(), cyl.getCenter().y());
    Eigen::Vector2d p1_xy(p1.x(), p1.y());
    Eigen::Vector2d p2_xy(p2.x(), p2.y());

    Eigen::Vector2d d = p2_xy - p1_xy;
    Eigen::Vector2d f = p1_xy - c_xy;

    double a = d.dot(d);
    double b = 2 * f.dot(d);
    double c = f.dot(f) - cyl.getR() * cyl.getR();

    double discriminant = b*b - 4*a*c;
    if (discriminant < 0) return -1; // no intersection

    discriminant = sqrt(discriminant);
    double t1 = (-b - discriminant) / (2*a);
    double t2 = (-b + discriminant) / (2*a);

    if ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1)) {
        // compute closest point along segment
        Eigen::Vector3d closest = p1 + (p2 - p1) * std::max(0.0, std::min(1.0, t1));
        double dz = std::max(0.0, std::abs(closest.z() - cyl.getCenter().z()) - cyl.getH()/2);
        return dz + (closest.head<2>() - c_xy).norm();
    }
    return -1;
}

double calcDiscriminant(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& center, double radius) {
    Eigen::Vector2d c_xy(center.x(), center.y());
    Eigen::Vector2d p1_xy(p1.x(), p1.y());
    Eigen::Vector2d p2_xy(p2.x(), p2.y());

    Eigen::Vector2d d = p2_xy - p1_xy;
    Eigen::Vector2d f = p1_xy - c_xy;

    double a = d.dot(d);
    double b = 2 * f.dot(d);
    double c = f.dot(f) - radius * radius;

    return b*b - 4*a*c; 
}

double Solution::computeCollisionCost(Eigen::Vector3d bounds, std::vector<Threat> threats) {
    // for each line segment
        // for each threat
            // compute discriminants and assign value
    double totalCollisionCost = 0.0;
    for (size_t i = 0; i < waypoints.size() - 1; i++) {
        for (Threat t : threats) {
            Eigen::Vector3d p1 = waypoints[i];
            Eigen::Vector3d p2 = waypoints[i-1];
            Eigen::Vector3d Ck = t.getCenter();
            double Rk = t.getR();
            double disc1 = calcDiscriminant(p1, p2, Ck, Rk + D + L);
            double disc2 = calcDiscriminant(p1, p2, Ck, Rk + D);

            if (disc1 < 0 && disc2 < 0) {
                // no collision
                totalCollisionCost += 0;
            } else if (disc1 > 0 && disc2 < 0) {
                // danger zone
                double dk = lineSegmentCylinderDistance(p1, p2, t);
                totalCollisionCost += Rk + D + L - dk;
            } else {
                // full collision
                // totalCollisionCost = std::numeric_limits<double>::infinity();
                // return totalCollisionCost;
                repairWaypoint('c', i, bounds, threats, std::vector<std::vector<int>> {});
            }

        } 
    }

    return totalCollisionCost;
}

double Solution::computeElevationCost(Eigen::Vector3d bounds, std::vector<std::vector<int>> heightmap) {
    double totalElevationCost = 0.0;
    const double hMax = bounds.z();

    // for each point
        // compute height penatly
    int index = 0;
    for (Eigen::Vector3d w : waypoints) {
        if (w.y() == heightmap.size())
            w.y() = heightmap.size() - 1;
        if (w.x() == heightmap.size())
            w.x() = heightmap.size() - 1;
        if (w.y() < 0)
            w.y() = 0;
        if (w.x() < 0)
            w.x() = 0;
        const double hMin = heightmap[static_cast<int>(w.y())][static_cast<int>(w.x())] + 10;

        if (w.z() > hMin && w.z() < hMax) {
            totalElevationCost += abs(w.z() - ((hMax + hMin) / 2));
        } else {
            // repair waypoint
            // totalElevationCost = std::numeric_limits<double>::infinity();
            // return totalElevationCost;
            repairWaypoint('e', index, bounds, std::vector<Threat>{}, heightmap);
        }
        index++;
    }

    return totalElevationCost;
}

double Solution::computeDistanceCost(Eigen::Vector3d bounds) {
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

double Solution::computeCost(Eigen::Vector3d bounds, std::vector<Threat> threats, std::vector<std::vector<int>> heightmap) {
    // distance cost
    double distanceCost = computeDistanceCost(bounds);
    // threat cost
    double collisionCost = computeCollisionCost(bounds, threats);
    // elevation cost
    double elevationCost = computeElevationCost(bounds, heightmap);
    // corner cost
    double cornerCost = computeCornerCost();

    double totalCost = bd * distanceCost + bc * collisionCost + be * elevationCost + bcorn * cornerCost;

    return totalCost;
}

void Solution::repairWaypoint(char domain, int waypointIndex, Eigen::Vector3d bounds, std::vector<Threat> threats, std::vector<std::vector<int>> heightmap) {
    // move every waypoint after this a certain x, y, or z ammount until that point is feasible
    // if elevation issue, move down
    // if collisionIssue, move in all axis
    Eigen::Vector3d w = waypoints[waypointIndex];
    int delta = 0;
    if (w.y() == heightmap.size())
            w.y() = heightmap.size() - 1;
        if (w.x() == heightmap.size())
            w.x() = heightmap.size() - 1;
        if (w.y() < 0)
            w.y() = 0;
        if (w.x() < 0)
            w.x() = 0;

    if (domain == 'e') {
        // elevation issue
        if (w.z() > bounds.z()) {
            // too high
            while (w.z() > heightmap[static_cast<int>(w.y())][static_cast<int>(w.x())] + 15) {
                w.z() -= 1;
                delta -= 1;
            }
        } else {
            // too low
            while (w.z() < heightmap[static_cast<int>(w.y())][static_cast<int>(w.x())] + 15) {
                w.z() += 1;
                delta += 1;
            }
        }
        // std::cout << w.z() << "\n";
        // while (w.z() < heightmap[static_cast<int>(w.y())][static_cast<int>(w.x())]) {
        //     std::cout << "mf how is this accepted" << w.z() << "\n";
        //     w.z() += 1;
        //     delta += 1;
        // }
        

        for (size_t i = waypointIndex+1; i < waypoints.size() - 1; i++) {
            waypoints[i].z() += delta;
        }

    } else if (domain == 'c') {
        return;
        // collision issue
        delta = 1;
        for (size_t i = waypointIndex+1; i < waypoints.size() - 1; i++) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0.0, 1.0);
            if(dis(gen) < 0.5) {
                if (dis(gen) < 0.6) {
                    waypoints[i].x() -= delta;
                } else {
                    waypoints[i].x() += delta;
                }
            } else {
                if (dis(gen) < 0.6) {
                    waypoints[i].y() -= delta;
                } else {
                    waypoints[i].y() += delta;
                }
            }
            if (waypoints[i].x() > bounds.x())
                waypoints[i].x() = bounds.x()-1;
            if (waypoints[i].x() < 0)
                waypoints[i].x() = 0;
            if (waypoints[i].y() > bounds.x())
                waypoints[i].y() = bounds.x()-1;
            if (waypoints[i].y() < 0)
                waypoints[i].y() = 0;

        }
    }
}


void Archive::sortByCost() {
    std::sort(solutions.begin(), solutions.end(), [] (const Solution& a, const Solution& b) {
        return a.cost < b.cost;
    });
}

void ACOSRAR::cycle(int curIter, int maxIter, std::vector<Threat> threats) {
    // Step 1: sort archive according to cost
    // for (int i = 0; i < archive->solutions.size(); i++) {
    //     std::cout << archive->solutions[i].cost << " ";
    // }
    // std::cout << "\n";
    archive->sortByCost();

    // Step 2 & 3 done in the constructor

    // Step 4: create X new paths to replace the worst X from the archive
        // Step 4.1: pick a guiding solution based off the probabilities

        // Improvement #1 select prob
            double q, q0;
            int guidingIndex = 0;
    
            // q is uniform random i believe
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0.0, 1.0);
            q = dis(gen);

            // compute q0
            const double k = curIter;
            const double K = maxIter;
            const double m = 0.2; // [0.1, 0.5]
            const double lambda = 1; // [0.1, 3] small = greedy, large = explore more

            q0 = m * exp(-1.0 * lambda * (k / K));
            if (q > q0) {
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

                guidingIndex = std::distance(CDF.begin(), it);
            } else {
                // choose max whihc means guidingIndex = 0 
                // this was written for redundancy and clarity
                std::vector<double>::iterator it = std::max_element(probs.begin(), probs.end());
                guidingIndex = std::distance(probs.begin(), it);
            }

            
 
        // Step 4.2: generate k solutions
        std::vector<Solution> newSolutions;
        for (size_t i = 0; i < archiveSize; i++) {
            Solution s;

            s = createNewSolution(archive->solutions[guidingIndex], guidingIndex);
            s.cost = s.computeCost(bounds, threats, heightmap);

            newSolutions.push_back(s);
        }
        // replace old solutions
        archive->solutions = std::move(newSolutions);
}

Solution ACOSRAR::createNewSolution(Solution &guide, int guideIndex) {
    Solution s;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    std::vector<double> r_vec(archive->solutions.size());
    for (size_t i = 0; i < archive->solutions.size(); ++i) {
        r_vec[i] = dist(gen);
    }

    s.waypoints.push_back(START);

    for (size_t i = 1; i < guide.waypoints.size() - 1; i++) { //first and last contain start and goal
        // assign a random number to each index

        /**
         * prob_random = prob_final + (prob_first - prob_final) * exp( -1.0 * mu / k), k = i
         */
        double pr = r_vec.back() + (r_vec.front() - r_vec.back()) * exp(-mu * i);

        /** if random number > pr jump
            x = x + std + levy(alpha),
            alpha = [0,2]
        */
        if (dist(gen) < pr) { 
            // calculate the std for each dim
            double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
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
        } else {
            auto levy = [&](double alpha_val) {
                std::normal_distribution<double> gauss(0.0, 1.0);
                double sigma_u = pow(
                    (tgamma(1 + alpha_val) * sin(M_PI * alpha_val / 2)) /
                    (tgamma((1 + alpha_val) / 2) * alpha_val * pow(2, (alpha_val - 1) / 2)),
                    1.0 / alpha_val
                );
                double u = gauss(gen) * sigma_u;
                double v = gauss(gen);
                return u / pow(abs(v), 1.0 / alpha_val);
            };

            double x_mod = guide.waypoints[i].x() + levy(alpha);
            x_mod = (x_mod > bounds.x()) ? bounds.x() : (x_mod < 0 ? 0 : x_mod);
            double y_mod = guide.waypoints[i].y() + levy(alpha);
            y_mod = (y_mod > bounds.y()) ? bounds.y() : (y_mod < 0 ? 0 : y_mod);
            double z_mod = guide.waypoints[i].z() + levy(alpha);
            z_mod = (z_mod > bounds.z()) ? bounds.z() : (z_mod < 0 ? 0 : z_mod);

            s.waypoints.push_back(Eigen::Vector3d(x_mod, y_mod, z_mod));
        }
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

        s.cost = s.computeCost(bounds, {}, heightmap);

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
    std::cout << "\ncalling pathfind\n";

    // pass acosrar bounds
    const int archiveSize = 15;
    const int numOfCycles = 350;
    ACOSRAR algo(startPos, goals[0], bounds, archiveSize, heightmap);
    algo.createInitialArchive();

    for (size_t i = 0; i < numOfCycles; i++) {
        algo.cycle(i+1, numOfCycles, threats);
        algo.archive->sortByCost();

        if (i % 10 == 0) {
            double progress = double(i) / numOfCycles;
            int width = 50, pos = width * progress;

            std::cout << "\r\033[1;32mBest Cost:\033[0m " << algo.archive->solutions[0].cost << " [";
            for (int j = 0; j < width; ++j)
                std::cout << (j < pos ? "=" : (j == pos ? ">" : " "));
            std::cout << "] " << int(progress * 100) << "%";
            std::cout.flush();
        }
    }

    double progress = 1;
    int width = 50, pos = width * progress;

    std::cout << "\r\033[1;32mBest Cost:\033[0m " << algo.archive->solutions[0].cost << " [";
    for (int j = 0; j < width; ++j)
        std::cout << (j < pos ? "=" : (j == pos ? ">" : " "));
    std::cout << "] " << int(progress * 100) << "%";
    std::cout.flush();

    // double lastbest = -1.0;
    // do {
    //     algo.cycle();
    //     algo.archive->sortByCost();
    //     std::cout << "Best Cost: " << algo.archive->solutions[0].cost << "\n";
    //     if (lastbest == algo.archive->solutions[0].cost) {
    //         algo.archive->solutions[0] = algo.createNewSolution(algo.archive->solutions[0], archiveSize-1);
    //     }
    //     lastbest = algo.archive->solutions[0].cost;
    // } while (algo.archive->solutions[0].cost > 750);

    algo.archive->sortByCost();

    path = std::move(algo.archive->solutions[0].waypoints);

    std::cout << "\ncalled pathfind\n";
}