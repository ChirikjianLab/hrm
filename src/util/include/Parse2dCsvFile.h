#ifndef PARSE2DCSVFILE_H
#define PARSE2DCSVFILE_H

#include <ompl/util/Time.h>
#include <fstream>
#include <iostream>
#include <vector>

std::vector<std::vector<double>> parse2DCsvFile(std::string inputFileName);

#endif  // PARSE2DCSVFILE_H
