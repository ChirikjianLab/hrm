#include "include/Parse2dCsvFile.h"

std::vector<std::vector<double>> parse2DCsvFile(const std::string& filename) {
    std::vector<std::vector<double>> data;
    std::ifstream inputFile(filename);
    int l = 0;

    while (inputFile) {
        l++;
        std::string s;
        if (!getline(inputFile, s)) {
            break;
        }
        if (s[0] != '#') {
            std::istringstream ss(s);
            std::vector<double> record;

            while (ss) {
                std::string line;
                if (!getline(ss, line, ',')) {
                    break;
                }
                try {
                    record.emplace_back(stof(line));
                } catch (const std::invalid_argument& e) {
                    std::cout << "NaN found in file " << filename << " line "
                              << l << std::endl;
                    e.what();
                }
            }

            data.push_back(record);
        }
    }

    if (!inputFile.eof()) {
        std::cerr << "Could not read file " << filename << "\n";
        std::__throw_invalid_argument("File not found.");
    }

    return data;
}
