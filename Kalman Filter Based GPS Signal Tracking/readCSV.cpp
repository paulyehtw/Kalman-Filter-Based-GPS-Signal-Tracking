//
//  readCSV.cpp
//  Kalman Filter Based GPS Signal Tracking
//
//  Created by Paul on 2018/9/25.
//  Copyright © 2018 Paul Yeh. All rights reserved.
//

#include "readCSV.hpp"
#include <sstream>
#include <Eigen>

readCSV::readCSV(string s)
{
    this->file_path = s;
}

Eigen::MatrixXd readCSV::CSVoutput()
{
    ifstream latlon(this->file_path);
    while ( getline(latlon,this->line) )    // get next line in file
    {
        this->v.clear();
        stringstream ss(this->line);
        
        while (getline(ss,this->field,','))  // break line into comma delimitted fields
        {
            cout.precision(10);
            this->v.push_back(stod(this->field));  // add each field to the 1D array
        }
        
        this->array.push_back(this->v);  // add the 1D array to the 2D array
    }
    
    Eigen::MatrixXd input(array.size(),2);
    for(int i = 0; i<this->array.size(); i++)
    {
        input(i,0) = this->array[i][0];
        input(i,1) = this->array[i][1];
    }
    return input;
}
