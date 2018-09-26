//
//  main.cpp
//  Kalman Filter Based GPS Signal Tracking
//
//  Created by Paul on 2018/9/25.
//  Copyright © 2018 Paul Yeh. All rights reserved.
//

#include <iostream>
#include <Eigen>
#include <fstream>
#include "readCSV.hpp"

using namespace std;
using namespace Eigen;


int main()
{
    float dt = 0.2;

    readCSV readdata("/Users/Paul/Documents/GitHub/Kalman Filter Based GPS Signal Tracking/Input/latlon.csv");
    ofstream path("/Users/Paul/Documents/GitHub/Kalman Filter Based GPS Signal Tracking/Output/KF_Path.txt");
    ofstream grundtruth("/Users/Paul/Documents/GitHub/Kalman Filter Based GPS Signal Tracking/Output/grundtruth.txt");
    ofstream noisyGPS("/Users/Paul/Documents/GitHub/Kalman Filter Based GPS Signal Tracking/Output/noisyGPS.txt");
    MatrixXd LatLon = readdata.CSVoutput();
    MatrixXd GoundTruth = LatLon;
    MatrixXd F(4,4);
    MatrixXd B(4,1);
    MatrixXd P(4,4);
    MatrixXd Z(2,1);
    MatrixXd y(2,1);
    MatrixXd S(2,2);
    MatrixXd K(2,2);
    MatrixXd R(2,2);
    MatrixXd Q(4,4);
    MatrixXd H(2,4);
    MatrixXd I(4,4);
    MatrixXd init(2,1);
    MatrixXd x(4,1);
    MatrixXd lon(5,1);
    MatrixXd lat(5,1);
    MatrixXd noise = MatrixXd::Random(LatLon.size()/2,2)/90000;
    LatLon = LatLon + noise;

    
    init << LatLon(0,0),LatLon(0,1);
    
    F << 1, 0, dt , 0,
         0, 1, 0, dt,
         0, 0, 1, 0,
         0, 0, 0, 1;
    
    B << pow(dt,2)/2, pow(dt,2)/2, dt, dt;
    
    float u = 0;
    
    P << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1000, 0,
    0, 0, 0, 1000;
    
    R << 0.1, 0,
    0, 0.1;
    
    Q << 0.001, 0, 0.001, 0,
         0, 0.001, 0, 0.001,
         0.001, 0, 0.001, 0,
         0, 0.001, 0, 0.001;
    
    H << 1, 0, 0, 0,
         0, 1, 0, 0;
    
    I.setIdentity();
    
    x << init(0), init(1), 0, 0;
    
    for(int i = 0; i<LatLon.size()/2; i++)
    {
        //prediction
        x = F * x + B*u;
        P = F * P * F.transpose() + Q;

        //measurement update
        Z << LatLon(i,0),LatLon(i,1);
        y = Z - H*x;
        S = H * P * H.transpose() + R;
        K = P * H.transpose() * S.inverse();
        x = x + (K * y);
        P = (I - (K * H)) * P;
        //        cout<<x(0)<<" "<<x(1)<<endl;
        path.precision(10);
        grundtruth.precision(10);
        noisyGPS.precision(10);
        path <<x(0,0)<<" "<<x(1,0)<<endl;
        noisyGPS<< Z(0,0)<<" "<<Z(1,0)<<endl;
        grundtruth<< GoundTruth(i,0)<<" "<<GoundTruth(i,1)<<endl;

    }
    return 0;
}
