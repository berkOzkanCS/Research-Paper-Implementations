#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>

using namespace std;

vector<vector<unsigned char>> readPGM(const string &filename, int &width, int &height, int &maxVal) {
    ifstream file(filename, ios::binary);
    if (!file) { cerr << "Cannot open file\n"; exit(1); }

    string line;
    file >> line;           // P5
    file >> width >> height;
    file >> maxVal;
    file.get();             // consume the single whitespace after maxVal

    vector<vector<unsigned char>> data(height, vector<unsigned char>(width));
    for(int i = 0; i < height; i++)
        file.read(reinterpret_cast<char*>(data[i].data()), width);

    return data;
}

void writePGM(const string &filename, const vector<vector<unsigned char>> &data, int maxVal) {
    ofstream file(filename, ios::binary);
    int height = data.size();
    int width = data[0].size();

    file << "P5\n" << width << " " << height << "\n" << maxVal << "\n";
    for(int i = 0; i < height; i++)
        file.write(reinterpret_cast<const char*>(data[i].data()), width);
}

void drawVerticalLine(vector<vector<unsigned char>> &data, int x, unsigned char value) {
    for(int i = 0; i < data.size(); i++)
        data[i][x] = value;
}

void drawHorizontalLine(vector<vector<unsigned char>> &data, int y, unsigned char value) {
    for(int j = 0; j < data[0].size(); j++)
        data[y][j] = value;
}

void blur(vector<vector<unsigned char>> &data, int &width, int &height, int kW) {
    for (int i = 0; i < kW / 2 + 1; i++) {
        int tempKW = kW / 2 + 1;
        double sum = 0;
        double divisor = 0;
        vector<vector<int>> k(height, vector<int>(tempKW + 1, 0));
        for (int row = 0; row < height; row++) {
            for (int j = 0; j < tempKW; j++) {
                k[row][j] = abs(kW - abs(i - j));
                divisor += k[row][j];
                sum += data[row][j] * k[row][j];
            }
        }
        double mean = sum / divisor;
        drawVerticalLine(data, i, mean);
    }

    for (int i = kW; i < width - (kW / 2 + 1); i++) {
        double sum = 0;
        double divisor = 0;
        vector<vector<int>> k(height, vector<int>(kW, 0));

        for (int row = 0; row < height; row++) {
            for (int j = 0; j < kW; j++) {
                k[row][j] = abs(kW - abs(kW - ((kW/2)+1) - j));
                divisor += k[row][j];
                sum += data[row][i - kW/2 + j] * k[row][j];
            }
        }
        double mean = sum / divisor;
        drawVerticalLine(data, i, mean);
    }

    for (int i = width - (kW / 2 + 1); i < width; i++) {

    }


}

int medianOfVector(vector<unsigned char> data) {
    size_t n = data.size() / 2;  
    std::nth_element(data.begin(), data.begin() + n, data.end());

    int median = data[n];
    return median;
}

int medianOfmedians(vector<vector<unsigned char>> &data, int start, int end, int &height, vector<double> &errors) {
    // cout << "S: " << start << " E: " << end << "\n";
    double error = 0.0;
    vector<unsigned char> mediansList;

    for (int i = 0; i < height; i++) {
        vector<unsigned char> row(data[i].begin() + start, data[i].begin() + end);
        // cout << "Row length: " << row.size() << "\n";

        int median = medianOfVector(row);
        mediansList.push_back(median);

        int sum = accumulate(row.begin(), row.end(), 0);
        error += (sum - static_cast<int>(row.size()) * median) / row.size();
    }
    // cout << error << "\n";
    errors.push_back(error);
    return medianOfVector(mediansList);
}

pair<vector<unsigned char>, vector<double>> mediansOfsectors(vector<vector<unsigned char>> &data, vector<unsigned int> hyperlineLocX, int &width, int &height) {
    vector<unsigned char> mediansList;
    vector<double> errors;
    const double GAMMA = 2;

    hyperlineLocX.push_back(width-1);
    hyperlineLocX.insert(hyperlineLocX.begin(), 0);

    for (size_t i = 1; i < hyperlineLocX.size()-1; i++) {
        int distL = abs(static_cast<int>(hyperlineLocX[i]) - static_cast<int>(hyperlineLocX[i-1]));
        int distR = abs(static_cast<int>(hyperlineLocX[i]) - static_cast<int>(hyperlineLocX[i+1]));
        int distD;
        if (distL > distR) {
            distD = distR / GAMMA;
        } else {
            distD = distL / GAMMA;
        }

        int start = hyperlineLocX[i] - distD;
        int end = hyperlineLocX[i] + distD;

        // cout << "Dummy Line: \t" << start << " <--- " << hyperlineLocX[i] << " ---> " << end << "\n";

        int median = medianOfmedians(data, start, end, height, errors);
        mediansList.push_back(median);
    }

    return {mediansList, errors};
}

int main_algorithm(int hyperline, int median_c1, int median_c2, int error_c1, int error_c2, int width) {
    string operation;
    char potential_c1, potential_c2;

    // determine potentials
    if (median_c1 > median_c2) {
        potential_c1 = '+';
        potential_c2 = '-';
    } else {
        potential_c1 = '-';
        potential_c2 = '+';
    }

    // choose operation
    if (potential_c1 == '+') {
        if (error_c1 > error_c2) {
            if (error_c1 < 0) operation = "LEFT";
            else operation = "RIGHT";
        } else {
            if (error_c2 < 0) operation = "LEFT";
            else operation = "RIGHT";
        }
    } else { // potential_c2 == '+'
        if (error_c1 > error_c2) {
            if (error_c1 < 0) operation = "RIGHT";
            else operation = "LEFT";
        } else {
            if (error_c2 < 0) operation = "RIGHT";
            else operation = "LEFT";
        }
    }

    // update hyperline based on operation
    if (operation == "LEFT" && hyperline > 0) {
        hyperline -= 1;
    } else if (operation == "RIGHT" && hyperline < width-1) {
        hyperline += 1;
    }

    return hyperline;
}


pair<vector<unsigned char>, vector<double>> segment(vector<vector<unsigned char>> &data, int width, int height, vector<unsigned int> &hyperlineLocX) {
    // get predefined hyperline columns
    // draw them onto the shape
    // for (size_t i = 0; i < hyperlineLocX.size(); i++) {
    //     drawVerticalLine(data, hyperlineLocX[i], abs(static_cast<int>(255)));
    // }

    // find medians of each sector
    return mediansOfsectors(data, hyperlineLocX, width, height);
}

void cycle(vector<vector<unsigned char>> &data, int width, int height, vector<unsigned int> &hyperlineLocX) {
    auto result = segment(data, width, height, hyperlineLocX);
    vector<unsigned char> medians = result.first;
    vector<double> errors = result.second;
    // for each hyper line run algorithm
    for (size_t i = 0; i < hyperlineLocX.size(); i++) {
        hyperlineLocX[i] = main_algorithm(hyperlineLocX[i], static_cast<int>(medians[i]), static_cast<int>(medians[i+1]), errors[i], errors[i+1], width);
    }

    cout << "HYL: \t";
    for (size_t i = 0; i < hyperlineLocX.size(); i++) {
        cout << hyperlineLocX[i] << " ";
    }
    cout << "\n";

}

int main() {
    int w, h, maxVal;
    auto image = readPGM("rcs.pgm", w, h, maxVal);
    auto imageSecond = readPGM("rcs.pgm", w, h, maxVal);
    vector<unsigned int> hyperlines = {280, 750, 820, 900, 1500, 1550, 2000};

    for (size_t i = 0; i < hyperlines.size(); i++) {
       drawVerticalLine(image, hyperlines[i], 255);
    }
    writePGM("rcs_initial.pgm", image, maxVal);

    // blur(image, w, h, 9);
    for (int i = 0; i < 10; i++) {
        cout << "Cycle: " << i << " ";
        cycle(image, w, h, hyperlines);
    }

    cout << "HYL AFTER: \t";
    for (size_t i = 0; i < hyperlines.size(); i++) {
       cout << hyperlines[i] << " ";
    }
    cout << "\n";

    for (size_t i = 0; i < hyperlines.size(); i++) {
       drawVerticalLine(imageSecond, hyperlines[i], 255);
    }
    writePGM("rcs_modified.pgm", imageSecond, maxVal);
}
