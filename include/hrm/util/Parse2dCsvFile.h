#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

namespace hrm {

std::vector<std::vector<double>> parse2DCsvFile(const std::string& filename);

}
