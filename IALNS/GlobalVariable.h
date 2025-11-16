#ifndef GLOBALVARIABLE_H
#define GLOBALVARIABLE_H

#include <vector>
#include <string>
using namespace std;

extern std::string SOLUTION;

//记录算子的权重
extern vector<double> kWeightRemoval;
extern vector<double> kWeightNoise;
extern vector<double> kWeightInsertion;

extern std::vector<std::vector<double>> kDist_D;//无人机距离矩阵
extern std::vector<std::vector<double>> kDist_T;//卡车距离矩阵
extern std::vector<double> kDemand;//需求矩阵
extern const int kNumCustomer;//客户数量
extern const double kDistMax_D;//最大距离
extern const double kDistMax_T;//最大距离
extern const double KDemandDifMax;//最大需求差
extern const double kDemandTotal;//总需求量

//记录算子一个步长内的得分
extern vector<vector<double>> points_i;
extern vector<vector<double>> points_n;
extern vector<vector<double>> points_r;

//记录边的权重
extern vector<vector<double>> edge_d;
extern vector<vector<double>> edge_t;

//记录k的权重
extern vector<double> weightK;

void initialGVariable();//初始化全局变量

#endif
