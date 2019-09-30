#ifndef PARSE2DCSVFILE_H
#define PARSE2DCSVFILE_H

#include <fstream>
#include <iostream>
#include <ompl/util/Time.h>
#include <vector>

std::vector<std::vector<double>> parse2DCsvFile(std::string inputFileName);

#endif // PARSE2DCSVFILE_H
