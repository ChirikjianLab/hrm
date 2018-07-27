#ifndef PARSE2DCSVFILE_H
#define PARSE2DCSVFILE_H

#include <iostream>
#include <fstream>
#include <vector>
#include <ompl/util/Time.h>

using namespace std;

class inputFile
{
public:
    vector<vector<double>> parse2DCsvFile(string inputFileName);
};


#endif // PARSE2DCSVFILE_H
