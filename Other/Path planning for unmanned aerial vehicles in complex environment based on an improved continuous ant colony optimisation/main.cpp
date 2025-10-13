#include "structures.cpp"


std::vector<std::vector<double>> readHeightMap(const std::string& filename) {
    std::vector<std::vector<double>> map;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open file: " << filename << "\n";
        return map;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::vector<double> row;
        std::istringstream ss(line);
        double h;
        while (ss >> h) {
            row.push_back(h);
        }
        map.push_back(row);
    }
    file.close();
    return map;
}

int main() {


    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> cR(80, 100); // cylinder radius
    std::uniform_int_distribution<int> cH(150, 400); // cylinder height
    std::uniform_int_distribution<int> locX(0, X_LIMIT);
    std::uniform_int_distribution<int> locY(0, Y_LIMIT);

    for (size_t i = 0; i < 3; i++) {
        Cylinder c = Cylinder{Vector3d(locX(rng), locY(rng), 0), static_cast<double>(cR(rng)), static_cast<double>(cH(rng))};
        THREATS.push_back(c);
    }

    std::ofstream obsOut("threats.txt");
    for (const auto &c : THREATS) {
        obsOut << c.center.x() << " " << c.center.y() << " " << c.center.z() << " " << c.R << " " << c.h << "\n";
    }
    obsOut.close();

    int numWaypoints = 100;
    int archiveSize = 30;

    Archive archive(archiveSize, numWaypoints);

    int K = 100;

    for (size_t i = 0; i < K; i++) {
        archive = ACOSRAR(archive, i, K);        
    }
    
    archive.sortByFitness();
    Solution best = archive.getBest();

    // after running the shit
    std::ofstream out("path_output.txt");
    for (const auto& wp : best.path.waypoints) {
        out << wp.x() << " " << wp.y() << " " << wp.z() << "\n";
    }
    out.close();
 



    return 0;
}