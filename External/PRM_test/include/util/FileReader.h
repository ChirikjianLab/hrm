#ifndef IS_FILEREADER_H
#define IS_FILEREADER_H

#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

class FileReader{
    public:
       vector<vector<double>> parse2DCsvFile(string inputFileName); 

};

#endif
