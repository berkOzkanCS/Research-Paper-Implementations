#include <iostream>
#include <string>
#include <limits>
#include <random>
#include <fstream>
#include <sstream>
#include <cmath>
#include <Eigen/Dense>


using Eigen::Vector3d;

const double X_LIMIT = 500;
const double Y_LIMIT = 500;
const double Z_LIMIT = 500;
const double H_MAX = 501;
const double D = 1;
const double L = 1;


Vector3d start(0, 0, 0);      
Vector3d goal(480, 480, 480);

struct Cylinder {
    Eigen::Vector3d center;
    double R;
    double h;
};

std::vector<Cylinder> THREATS;
std::vector<std::vector<double>> MAP(Y_LIMIT, std::vector<double>(X_LIMIT, 0.0));


class Path {
    public:
        std::vector<Vector3d> waypoints;

        Path() = default;

        Path(const std::vector<Vector3d> &wpts) : waypoints(wpts) {}

        void addWaypoint(double x, double y, double z) {
            waypoints.push_back(Vector3d(x,y,z));
        }

        double distanceCost() {
            double totalDistanceCost = 0.0;
            for (size_t i = 0; i < waypoints.size() - 1; i++) {
                totalDistanceCost += (waypoints[i+1]-waypoints[i]).norm();
            }
            return totalDistanceCost;
        }

        double shortestDistPointToSegment(Vector3d &start, Vector3d &end, const Vector3d &point) {
            Vector3d line = end - start;
            float lineLength = line.squaredNorm();
            if (lineLength == 0.0f) return (point - start).squaredNorm();
            float t = (point - start).dot(line) / lineLength;
            t = std::clamp(t, 0.0f, 1.0f);
            Vector3d closest = start + t * line;
            return (point - closest).squaredNorm();
        }

        double discriminant(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Cylinder& cyl, double extraRadius) {
            Eigen::Vector3d d = p1 - p0;             
            Eigen::Vector3d m = p0 - cyl.center;

            double dx = d.x();
            double dy = d.y();
            double mx = m.x();
            double my = m.y();

            double R = cyl.R + extraRadius;

            double a = dx*dx + dy*dy;
            double b = 2.0 * (mx*dx + my*dy);
            double c = mx*mx + my*my - R*R;

            double disc = b*b - 4*a*c;

            // Clamp tiny negative discriminants
            if (disc < 0 && disc > -1e-8) disc = 0.0;

            // Normalize to avoid huge values by scaling relative to a
            if (a > 0) disc /= (a * 1e6); // scale down for stability

            return disc;
        }

        double collisionCheck(Vector3d &start, Vector3d &end, std::vector<Cylinder> &threats, double D, double L) {
            double threatCost = 0.0;

            for (const auto& cyl : threats) {
                // Skip if segment completely above or below cylinder
                double zMin = std::min(start.z(), end.z());
                double zMax = std::max(start.z(), end.z());
                if (zMax < cyl.center.z() || zMin > cyl.center.z() + cyl.h) {
                    continue; // safe, no threat
                }

                double discFi1 = discriminant(start, end, cyl, D + L);
                double discFi2 = discriminant(start, end, cyl, D);

                // Clamp tiny negative discriminants
                if (discFi1 < 0 && discFi1 > -1e-8) discFi1 = 0.0;
                if (discFi2 < 0 && discFi2 > -1e-8) discFi2 = 0.0;

                // std::cout << "d1: " << discFi1 << " d2: " << discFi2 << "\n";

                // Treat large positive discriminants using distance to cylinder instead of infinity
                if (discFi1 <= 0 && discFi2 <= 0) {
                    // std::cout << "tc is none\n";
                    continue; // no threat
                }

                // For any intersection (discFi1>0 or discFi2>0), compute finite threat cost
                // std::cout << "tc is finite\n";
                double dk = shortestDistPointToSegment(start, end, cyl.center);
                double cost = cyl.R + L + D - dk;
                if (cost > 0) threatCost += cost; // only add positive cost
            }

            return threatCost;
        }


        double collisionCost(std::vector<Cylinder> &threats, double D, double L) {
            double totalThreatCost = 0.0;
            for (size_t i = 0; i < waypoints.size() - 1; i++) {
                // std::cout << "checking points: " << waypoints[i].x() << ", " << waypoints[i].y() << ", " << waypoints[i].z() << ", " << waypoints[i+1].x() << ", " << waypoints[i+1].y() << ", " << waypoints[i+1].z() << "\n";
                totalThreatCost += collisionCheck(waypoints[i], waypoints[i+1], threats, D, L);
            }
            return totalThreatCost;
        }

        double altitudeCost(std::vector<std::vector<double>> map, double hMax) {
            double totalAltitudeCost = 0.0;
            for (size_t i = 0; i < waypoints.size(); i++) {
                // std::cout << "xyz: " << waypoints[i].x() << ", " << waypoints[i].y() << ", " << waypoints[i].z() << "\n";
                double height = waypoints[i].z() + map[static_cast<int>(waypoints[i].x())][static_cast<int>(waypoints[i].y())];
                double hMin = map[static_cast<int>(waypoints[i].x())][static_cast<int>(waypoints[i].y())];
                // std::cout << "hM - h - hMx: " << hMin << " " << height << " " << hMax << "\n";
                if (height + 1 > hMin && height < hMax) {
                    totalAltitudeCost += abs(height - (hMax+hMin)/2);
                } else {
                    // std::cout << "altitude cost is infinity because: hMin, h, hMax: " << hMin << " " << height << " " << hMax << "\n";
                    totalAltitudeCost = std::numeric_limits<double>::infinity();
                    return totalAltitudeCost;
                }
            }
            return totalAltitudeCost;
        }

        double cornerCost() {
            const double gamma1 = 1.0;
            const double gamma2 = 1.0;
            double totalAlpha = 0.0;
            double totalBetaDiff = 0.0;

            std::vector<double> betas;

            for (size_t i = 0; i < waypoints.size() - 2; i++) {
                Eigen::Vector3d v1 = waypoints[i+1] - waypoints[i];
                Eigen::Vector3d v2 = waypoints[i+2] - waypoints[i+1];

                Eigen::Vector3d v1ground = v1;
                Eigen::Vector3d v2ground = v2;
                v1ground.z() = 0;
                v2ground.z() = 0;

                double alpha = std::atan2(v1ground.cross(v2ground).norm(), v1ground.dot(v2ground));
                totalAlpha += alpha;

                Eigen::Vector3d seg = waypoints[i+1] - waypoints[i];
                Eigen::Vector3d seg_xy = seg;
                seg_xy.z() = 0;

                double beta = (waypoints[i+1].z() - waypoints[i].z()) / seg_xy.norm();
                betas.push_back(beta);
            }

            // sum vertical differences |beta_j - beta_{j-1}|
            for (size_t j = 1; j < betas.size(); j++) {
                totalBetaDiff += std::abs(betas[j] - betas[j-1]);
            }

            double totalCornerCost = gamma1 * totalAlpha + gamma2 * totalBetaDiff;
            return totalCornerCost;
        }

        double flightCost(double bdistance, double bcollision, double baltitude, double bcorner, std::vector<Cylinder> &threats, double D, double L, std::vector<std::vector<double>> map, double hMax) {
            double totalFlightCost = 0.0;

            double distnaceCosts = distanceCost();
            // std::cout << "distance: " << distnaceCosts << "\t";
            double collisionCosts = collisionCost(threats, D, L); // this one
            // std::cout << "collision: " << collisionCosts << "\t";
            // double altitudeCosts = altitudeCost(map, hMax); // this one
            double altitudeCosts = 0;
            // std::cout << "altitude: " << altitudeCosts << "\t";
            double cornerCosts = cornerCost();
            // std::cout << "corner: " << cornerCosts << "\t";

            totalFlightCost = bdistance * distnaceCosts + bcollision * collisionCosts + baltitude * altitudeCosts + bcorner * cornerCosts;
            return totalFlightCost;
        }

        void printWaypoints() {
            for (size_t i = 0; i < waypoints.size(); ++i) {
                const auto& wp = waypoints[i];
                std::cout << "Waypoint " << i << ": "
                        << wp.x() << " " << wp.y() << " " << wp.z() << "\n";
            }
        }

};

class Solution {
public:
    Path path;
    double fitness;
    double weight;
    double probability;
    
    Solution() : path(), fitness(0.0), weight(0.0) {}
    Solution(const Path& p, double f) : path(p), fitness(f), weight(0.0), probability(0.0) {}
    Solution(const Path& p, double f, double w) : path(p), fitness(f), weight(w), probability(0.0) {}
};

double bdistance = 1.0;
double bcollision = 10.0;
double baltitude = 10.0;
double bcorner = 5.0;

class Archive {
public:
    std::vector<Solution> solutions;
    double totalWeightSum = 0.0;
    
    Archive (int archiveSize, int numOfWaypoints) {
        static std::mt19937 rng(std::random_device{}());
        std::uniform_int_distribution<int> locX(0, X_LIMIT);
        std::uniform_int_distribution<int> locY(0, Y_LIMIT);
        std::uniform_int_distribution<int> locZ(0, Z_LIMIT);
    
        for (size_t i = 0; i < archiveSize; i++) {
            Solution s;
            s.path.waypoints.clear();
            s.path.addWaypoint(start.x(), start.y(), start.z());

            for (size_t j = 0; j < numOfWaypoints; j++) {
                s.path.addWaypoint(locX(rng), locY(rng), locZ(rng));
            }

            s.path.addWaypoint(goal.x(), goal.y(), goal.z());

            s.fitness = s.path.flightCost(bdistance, bcollision, baltitude, bcorner, THREATS, D, L, MAP, H_MAX);

            solutions.push_back(s);
        }

        std::cout << "generated\n";

    }

    void addSolution(const Solution& sol) {
        solutions.push_back(sol);
    }

    // Sort solutions descending by fitness (lower fitness = better)
    void sortByFitness() {
        std::sort(solutions.begin(), solutions.end(),
                  [](const Solution& a, const Solution& b) {
                      return a.fitness < b.fitness;
                  });
    }

    Solution getBest() const {
        return solutions.front();
    }

};


std::random_device rd;
std::mt19937 gen(rd());

double uniform(double a, double b) {
    std::uniform_real_distribution<> dis(a, b);
    return dis(gen);
}

double gaussian(double mean, double stddev) {
    std::normal_distribution<> dis(mean, stddev);
    return dis(gen);
}

// Lévy flight generator
double levy(double alpha, std::mt19937 &gen) {
    std::normal_distribution<double> gauss(0.0, 1.0);

    double u = gauss(gen);
    double v = gauss(gen);

    // Mantegna’s algorithm
    double sigma_u = pow((tgamma(1+alpha) * sin(M_PI*alpha/2)) /
                         (tgamma((1+alpha)/2) * alpha * pow(2, (alpha-1)/2)), 1.0/alpha);

    double step = u * sigma_u / pow(fabs(v), 1.0/alpha);
    return step;
}

double computePr(double p0, double pf, double mu, int k) {
    return pf + (p0 - pf) * exp(-mu * k);
}

double stdOfArchive(Archive &a, double p, char dim, int index) {
    double sum = 0.0;
    for (size_t i = 0; i < a.solutions.size(); i++) {
        if (dim == 'x') {
            sum += abs(p-a.solutions[i].path.waypoints[index].x()) / (a.solutions.size() - 1);
        } else if (dim == 'y') {
            sum += abs(p-a.solutions[i].path.waypoints[index].y()) / (a.solutions.size() - 1);
        } else {
            sum += abs(p-a.solutions[i].path.waypoints[index].z()) / (a.solutions.size() - 1);
        }
    }
    return sum;
}



bool isFeasible(const Eigen::Vector3d &wp, const std::vector<Cylinder> &threats) {
    for (const auto &cyl : threats) {
        // horizontal distance from center (x,y plane)
        double dx = wp.x() - cyl.center.x();
        double dy = wp.y() - cyl.center.y();
        double distXY = std::sqrt(dx*dx + dy*dy);

        bool insideRadius = distXY < cyl.R;
        bool insideHeight = (wp.z() >= cyl.center.z()) && (wp.z() <= cyl.center.z() + cyl.h);

        if (insideRadius && insideHeight) {
            return false; // inside obstacle
        }
    }
    return true;
}
const double X_MIN = 0.0, X_MAX = 10.0;
const double Y_MIN = 0.0, Y_MAX = 10.0;
const double Z_MIN = 0.0, Z_MAX = 5.0;
double randomFeasibleX(const Eigen::Vector3d &wp, const std::vector<Cylinder> &threats, std::mt19937 &gen) {
    std::uniform_real_distribution<> dis(X_MIN, X_MAX);
    Eigen::Vector3d candidate = wp;
    do {
        candidate.x() = dis(gen);
    } while (!isFeasible(candidate, threats));
    return candidate.x();
}

double randomFeasibleY(const Eigen::Vector3d &wp, const std::vector<Cylinder> &threats, std::mt19937 &gen) {
    std::uniform_real_distribution<> dis(Y_MIN, Y_MAX);
    Eigen::Vector3d candidate = wp;
    do {
        candidate.y() = dis(gen);
    } while (!isFeasible(candidate, threats));
    return candidate.y();
}

double randomFeasibleZ(const Eigen::Vector3d &wp, const std::vector<Cylinder> &threats, std::mt19937 &gen) {
    std::uniform_real_distribution<> dis(Z_MIN, Z_MAX);
    Eigen::Vector3d candidate = wp;
    do {
        candidate.z() = dis(gen);
    } while (!isFeasible(candidate, threats));
    return candidate.z();
}

void repairWaypoints(Path &p, const std::vector<Cylinder> &threats, std::mt19937 &gen) {
    std::uniform_real_distribution<> dis(0.0, 1.0);
    double c1 = 0.33, c2 = 0.66; // probability thresholds for x,y,z moves

    for (auto &wp : p.waypoints) {
        if (!isFeasible(wp, threats)) {
            double r = dis(gen);
            if (r < c1) {
                wp.x() = randomFeasibleX(wp, threats, gen);
            } else if (r < c2) {
                wp.y() = randomFeasibleY(wp, threats, gen);
            } else {
                wp.z() = randomFeasibleZ(wp, threats, gen);
            }
        }
    }
}


Archive ACOSRAR(Archive &a, int currentIter, int maxIter) {
    std::cout << "loop: " << currentIter << "/" << maxIter << "\n";
    double l = 1.0;
    const double xi = 1.0;
    int K = maxIter;
    int k = currentIter+1;
    double m = 0.8;
    const double mu = 0.05;
    double alpha = 1.5;

    a.sortByFitness();
    
    for (size_t i = 0; i < a.solutions.size(); i++) {
        std::cout << a.solutions[i].fitness << " ";
        double coeff = 1 / (l * a.solutions.size() * sqrt(2*M_PI));
        double expo = exp( - ( pow((i), 2) / (2 * pow(l, 2) * pow(a.solutions.size(), 2) ) ));
        a.solutions[i].weight = coeff * expo;
        a.totalWeightSum += a.solutions[i].weight;
    }
    std::cout << std::endl;

    std::vector<double> psi;
    for (size_t i = 0; i < a.solutions.size(); i++) {
        double prob = a.solutions[i].weight / a.totalWeightSum;
        psi.push_back(prob);
    }

    // randomly picking a guiding path
    // Improved state transition probability --------------------------------------


    double q0 = m * std::exp(-static_cast<double>(k) / K);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    double q = dis(gen);

    int guidingIndex = 0;
    if ( q < q0) {
        auto it = std::max_element(psi.begin(), psi.end());
        guidingIndex = std::distance(psi.begin(), it);
    } else {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::discrete_distribution<> dist(psi.begin(), psi.end());
        guidingIndex = dist(gen);
    }
    // ----------------------------------------------------------------------------
    
    double p0 = psi.front(); // first element
    double pf = psi.back();  // last element
    double pr = p0 + (pf - p0) * std::exp(-mu * k);

    Path newPath = a.solutions[guidingIndex].path;
    for (size_t i = 0; i < newPath.waypoints.size(); i++) {
        double stdX = xi * stdOfArchive(a, newPath.waypoints[i].x(), 'x', i);
        double stdY = xi * stdOfArchive(a, newPath.waypoints[i].y(), 'y', i);
        double stdZ = xi * stdOfArchive(a, newPath.waypoints[i].z(), 'z', i);

    // Random walk strategy --------------------------------------------------------
    
        if (uniform(0.0, 1.0) < pr) {
            newPath.waypoints[i].x() = gaussian(newPath.waypoints[i].x(), stdX);
            newPath.waypoints[i].y() = gaussian(newPath.waypoints[i].y(), stdY);
            newPath.waypoints[i].z() = gaussian(newPath.waypoints[i].z(), stdZ);
        } else {
            newPath.waypoints[i].x() += stdX * levy(alpha, gen);
            newPath.waypoints[i].y() += stdY * levy(alpha, gen);
            newPath.waypoints[i].z() += stdZ * levy(alpha, gen);
        }

    // -----------------------------------------------------------------------------
    
    }
 
    // Adaptive waypoint-repair step -----------------------------------------------
    // repairWaypoints(newPath, THREATS, gen);
    std::cout << "repaired waypoints\n"; 
    // -----------------------------------------------------------------------------

    // Evaluate fitness after repair
    double f = newPath.flightCost(bdistance, bcollision, baltitude, bcorner, THREATS, D, L, MAP, H_MAX);

    if (f < a.solutions.back().fitness) {
        a.solutions.back() = Solution(newPath, f, 0.0);
    }
    return a;
}
