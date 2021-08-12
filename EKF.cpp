#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include "matplotlibcpp.h"
#include <math.h>
#include<map>
#include<Read_data.cpp>
namespace plt = matplotlibcpp;
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
string sensor_file_name = "/home/avii/Documents/SLAM/data/data/sensor_data.dat";
string landmarks_file = "/home/avii/Documents/SLAM/data/data/world.dat";
using namespace Read_data;


vector<long double> map_limits{-1,12,-1,10};
vector<VectorXd> sigmDs;
vector<int> numMeasurements;


Eigen::Vector3d mu(0.0,0.0,0.0);
VectorXd Z;
VectorXd Ht;
MatrixXd sigma = MatrixXd::Identity(3,3);

MatrixXd Q(3,3);
MatrixXd G(3,3);
MatrixXd H_row(1,3);
MatrixXd H_prev;
MatrixXd H;
MatrixXd R;
MatrixXd S;
MatrixXd K;

long double normalize_angle(long double &phi)
{
   
    while(phi>M_PIf128) phi = phi - 2*M_PIf128;
    while(phi<-M_PIf128) phi = phi + 2*M_PIf128;
    return phi;
}
int count_measurements(vector<long double> ids)
{
    int count;
    count = ids.size();
    return count;
}

void plot_state(vector<long double> mu, MatrixXd sigma,map<int, 
                vector<int>> landmarks,vector<long double> map_limits,
                std::map<std::string,std::vector<long double>> sensor_data) {
    


}

void Predict(vector<long double> odometry)
{
    long double d_rot1 = odometry[0];
    long double d_rot2 = odometry[1];
    long double d_trans = odometry[2];
    long double x = mu[0];
    long double y = mu[1];
    long double theta = mu[2];

    Q << 0.2, 0.0 ,0.0,
         0.0, 0.2, 0.0,
         0.0, 0.0, 0.2;
    long double x_new = x + d_trans * cosl(theta + d_rot1);
    long double y_new = y + d_trans * sinl(theta + d_rot2);
    long double theta_new = theta + d_rot1 + d_rot2;
    //cout<<"x_new = "<<x_new<<" "<<y_new<<" "<<theta_new<<endl;
    G << 1.0, 0.0, -d_trans * sinl(theta + d_rot1),
         0.0, 1.0,  d_trans * cosl(theta + d_rot2),
         0.0, 0.0, 1.0;
    mu << x_new, y_new, theta_new;
   // cout<<" mu = "<<mu<<endl;
    sigma = G*(sigma*G.transpose()) + Q; 
   // cout<<"reached"<<endl;   
}

void Correct(map<std::string,std::vector<long double>> sensor_data, map<int, vector<int>> landmarks)
{

    long double x = mu[0];
    long double y = mu[1];
    long double theta = mu[2];

    vector<long double> ids = sensor_data["id"];
    vector<long double> ranges = sensor_data["range"];
    int numMeasurements = ids.size();
    vector<long double> z;
    vector<long double> ht;
    vector<MatrixXd> H_temp;
    for(int i = 0;i<ids.size();i++)
    {
            long double lm_id = ids[i];
            long double measured_range = ranges[i];
            int lx = landmarks[int(lm_id)][0];
            int ly = landmarks[int(lm_id)][1];

            long double exp_range = sqrtl(pow(lx - x, 2) + pow(ly - y, 2));
            H_row <<  (x-lx) / exp_range , (y-ly) / exp_range ,0.0 ;
            H_temp.push_back(H_row);
            // H = MatrixXd(H.rows()+1,H_row.cols());
            // H.resize(H.rows() + H_row.rows(),H_row.cols());
            // H << H,H_row;
            z.push_back(ranges[i]);
            ht.push_back(exp_range);
    }
    H = MatrixXd(H_temp.size(),H_row.cols());
    for(int i = 0;i< H.rows();i++)
    {
        for(int j = 0;j<H.cols();j++)
        {
            H(i,j) = H_temp[i](j);
        }
    }
    
    Z = VectorXd(z.size());
    Ht = VectorXd(ht.size());
    for(int i = 0;i<z.size();i++) 
    {
        Z(i) = z[i];
        Ht(i) = ht[i];
    }
    
    R = MatrixXd::Identity(ids.size(),ids.size()) * 0.5;  // Noise covariance
    
    S = H*sigma*H.transpose() + R;
    
    K = sigma*H.transpose()*S.inverse();
    
    mu = mu + K*(Z-Ht);
    
    
    sigma = (MatrixXd::Identity(sigma.rows(),sigma.cols()) - K*H)*sigma;
    
}





int main()
{
std::map<int,std::vector<long double>> Odometry_data;
std::map<int,std::map<std::string,std::vector<long double>>> sensor_data;
map<int, vector<int>> landmarks;
landmarks = Read_data::get_landmarks(landmarks_file);  //cout<<landmarks.size()<<endl;
Odometry_data = Read_data::get_odometry(sensor_file_name);  //cout<<Odometry_data.size()<<endl;
sensor_data = Read_data::get_sensor(sensor_file_name);  //cout<<sensor_data.size()<<endl;

for(int i = 0;i<sensor_data.size();i++)
{

    Predict(Odometry_data[i]);
    Correct(sensor_data[i],landmarks);
    //Read_data::Plot_State();
}
cout<<sigma<<endl;

return 0;
}
///home/Dhaval/Documents/SLAM/Kalman_filter/EKF.cpp