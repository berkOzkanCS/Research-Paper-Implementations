#include "sim.hpp"

void CylinderStrategy::collisionCheck(Eigen::Vector3d v) {

}

void DomeStrategy::collisionCheck(Eigen::Vector3d v) {
    
}

Threat::Threat(Eigen::Vector3d c, int r, int h)
    : center(c), R(r), H(h) {}


Eigen::Vector3d Threat::getCenter() const {
    return center; 
}

void Threat::setCenter(const Eigen::Vector3d& c) { 
    center = c; 
}

Map::Map()
    : bounds(Eigen::Vector3d(0, 0, 0)),
      threats(),
      startPos(Eigen::Vector3d(0, 0, 0)),
      goals()
{}

Map::Map(Eigen::Vector3d b,
         std::vector<Threat> t,
         Eigen::Vector3d sP,
         std::vector<Eigen::Vector3d> g)
    : bounds(b),
      threats(std::move(t)),
      startPos(sP),       
      goals(std::move(g))
{
    bounds.z() = 255;
}

Map::Map(Eigen::Vector3d b,
         Eigen::Vector3d sP,
         std::vector<Eigen::Vector3d> g,
         int numberOfThreats)
    : bounds(b),
      startPos(sP),      
      goals(std::move(g))
{
    bounds.z() = MAP_Z;
    std::random_device rd;
    std::mt19937 rng(rd());

    if (numberOfThreats < 0) {
        std::uniform_int_distribution<int> dist(5, 10);
        numberOfThreats = dist(rng);  
    }

    numberOfThreats = 0;

    std::uniform_real_distribution<double> ff(0.0, 1.0);
    std::uniform_real_distribution<double> centerX(0.0, bounds.x());
    std::uniform_real_distribution<double> centerY(0.0, bounds.y());
    std::uniform_int_distribution<int> Rdist(((bounds.x()+bounds.y())/2)*0.08, ((bounds.x()+bounds.y())/2)*0.1);
    std::uniform_int_distribution<int> Hdist(bounds.z()*0.35, bounds.z()*0.52);

    for (int i = 0; i < numberOfThreats; ++i) {
        Eigen::Vector3d pos(centerX(rng), centerY(rng), 0);
        int radius = Rdist(rng);
        int height = Hdist(rng);

        std::unique_ptr<GeneratorStrategy> strat;
        if (ff(rng) < 0.5)
            strat = std::make_unique<CylinderStrategy>();
        else
            strat = std::make_unique<DomeStrategy>();

        Threat t(pos, radius, height);
        threats.push_back(std::move(t));
    }
}

void Map::setPath(std::vector<Eigen::Vector3d> p) {
    path = p;
}

std::vector<Eigen::Vector3d> Map::pathGenerator(int numOfNodes, char setAxis) {
    static std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> locX(10, bounds.x());
    std::uniform_int_distribution<int> locY(10, bounds.y());
    std::uniform_int_distribution<int> locZ(10, bounds.z());

    const int setAxisValue = 365;
    path.clear();
    for (size_t i = 0; i < numOfNodes; i++) {
        int x = locX(rng);
        int y = locY(rng);
        int z = locZ(rng);
        if (setAxis == 'x' || setAxis == 'X') {
            x = setAxisValue;
        } else if (setAxis == 'y' || setAxis == 'Y') {
            y = setAxisValue;
        } else if (setAxis == 'z' || setAxis == 'Z') {
            z = setAxisValue;
        }
        
        path.push_back(Eigen::Vector3d(x,y,z));
    }
    return path;
}

void Map::printPath() {
    std::cout << "Path:\t";
    for (size_t i = 0; i < path.size(); i++) {
        std::cout << "(" << path[i].x() << ", " << path[i].y() << ", " << path[i].z() << ") ";
    }
    std::cout << "\n";
}

void Map::readMap(const std::string& filePath, int mapWidth, int mapHeight) {
    std::ifstream file(filePath, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Failed to open heightmap file: " + filePath);
    }

    std::vector<uint16_t> buffer(mapWidth * mapHeight, 0);
    file.read(reinterpret_cast<char*>(buffer.data()), buffer.size() * sizeof(uint16_t));


    // if (!file) {
    //     std::cout << std::filesystem::current_path() << std::endl;
    //     std::cout << filePath << std::endl;
    //     throw std::runtime_error("Failed to read full heightmap data from: " + filePath);
    // }

    heightmap.assign(mapHeight, std::vector<int>(mapWidth, 0));
    for (int y = 0; y < mapHeight; ++y) {
        for (int x = 0; x < mapWidth; ++x) {
            int idx = y * mapWidth + x;
            heightmap[y][x] = static_cast<int>(buffer[idx]);
            // std::cout << "height[" << y << "][" << x << "]: " << heightmap[y][x] << std::endl;
        }
    }

    for (auto& threat : threats) {
        Eigen::Vector3d c = threat.getCenter();
        int xi = static_cast<int>(c.x());
        int yi = static_cast<int>(c.y());

        if (xi >= 0 && xi < mapWidth && yi >= 0 && yi < mapHeight) {
            // std::cout << "height[" << yi << "][" << xi << "]: " << heightmap[yi][xi] << " + " << c.z() << " + " << threat.getH()/2 << "\n";
            c.z() = heightmap[yi][xi] + c.z() + threat.getH()/2;
        } else {
            c.z() = 0;
            // std::cout << "height[" << yi << "][" << xi << "]: " << heightmap[yi][xi] << " + " << c.z() << " + " << "\n";
        }

        // std::cout << "xi: " << xi << " yi: " << yi 
        //           << " hm[xy]: " << heightmap[yi][xi] 
        //           << " cz-h/2 " << c.z() - threat.getH()/2 << "\n";

        threat.setCenter(c);
    }
}

void Map::saveToFile(const std::string& filePath) const {
    std::ofstream out(filePath);
    if (!out) throw std::runtime_error("Cannot open file for writing: " + filePath);

    // Write bounds
    out << bounds.x() << " " << bounds.y() << " " << bounds.z() << "\n";

    // Write startPos
    out << startPos.x() << " " << startPos.y() << " " << startPos.z() << "\n";

    // Write goals
    out << goals.size() << "\n";
    for (const auto& g : goals)
        out << g.x() << " " << g.y() << " " << g.z() << "\n";

    // Write heightmap size
    int rows = heightmap.size();
    int cols = (rows > 0) ? heightmap[0].size() : 0;
    out << rows << " " << cols << "\n";

    // Write heightmap
    // for (const auto& row : heightmap) {
    //     for (int h : row) out << h << " ";
    //     out << "\n";
    // }

    // Write threats
    out << threats.size() << "\n";
    for (const auto& t : threats) {
        Eigen::Vector3d c = t.getCenter();
        out << c.x() << " " << c.y() << " " << c.z() << " "
            << t.getR() << " " << t.getH() << "\n";
    }

    for (const auto& p : path) {
        out << p.x() << " " << p.y() << " " << p.z() << "\n";
    }
}

