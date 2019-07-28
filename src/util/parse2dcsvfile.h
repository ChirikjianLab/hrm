#ifndef PARSE2DCSVFILE_H
#define PARSE2DCSVFILE_H

#include <fstream>
#include <iostream>
#include <ompl/util/Time.h>
#include <vector>

using namespace std;

class inputFile {
public:
  vector<vector<double>> parse2DCsvFile(string inputFileName);
};

#endif // PARSE2DCSVFILE_H
