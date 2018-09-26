//
//  readCSV.hpp
//  Kalman Filter Based GPS Signal Tracking
//
//  Created by Paul on 2018/9/25.
//  Copyright © 2018 Paul Yeh. All rights reserved.
//

#ifndef readinput_hpp
#define readinput_hpp

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen>

using namespace std;
class readCSV
{
    string file_path;
    string line,field;
    vector<vector<double>> array;  // the 2D array
    vector<double> v;              // array of values for one line only
    
public:
    readCSV(string s);
    Eigen::MatrixXd CSVoutput();
    
};

#endif /* readinput_hpp */
