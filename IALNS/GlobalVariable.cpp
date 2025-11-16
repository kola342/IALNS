#include "GlobalVariable.h"
#include "CustomerInfo.h"

//数据文件
std::string FILENAME_XY = "D:\\Document\\VS_code\\TSPD1_data\\r_16_0.15.txt";//客户坐标文件地址(最后一点为原点)
std::string FILENAME_DEMAND = "D:\\Document\\VS_code\\TSPD1_data\\demand_16_12.txt";//客户需求量地址(不包含原点)
std::string SOLUTION = "D:\\Document\\VS_code\\TSPD1_data\\solution_r_16.txt";


std::vector<std::vector<double>> kDist_D = { {0} };//无人机距离矩阵
std::vector<std::vector<double>> kDist_T = { {0} };//卡车距离
std::vector<double> kDemand = { 0 };//需求矩阵
vector<double> kWeightRemoval = vector<double>(6, OPERATOR_D);//移除算子的权重
vector<double> kWeightNoise = vector<double>(2, OPERATOR_D);//噪声算子的权重
vector<double> kWeightInsertion = vector<double>(3, OPERATOR_D);//插入算子的权重
vector<vector<double>> points_i = vector<vector<double>>(kWeightInsertion.size(), vector<double>(2, 0));//移除算子的得分
vector<vector<double>> points_n = vector<vector<double>>(kWeightNoise.size(), vector<double>(2, 0));//噪声算子的得分
vector<vector<double>> points_r = vector<vector<double>>(kWeightRemoval.size(), vector<double>(2, 0)); //插入算子的得分
vector<double> weightK = vector<double>(REGRET_K, OPERATOR_D);//k的权重

void initialGVariable() {}

//给全局常量进行赋值
const int kNumCustomer = []() {
    CustomerInfo customer_info(FILENAME_XY, FILENAME_DEMAND);//读取数据
    kDist_D = customer_info.matrixDist_D();//读取无人机距离矩阵
    kDist_T = customer_info.matrixDist_T();//读取卡车距离矩阵
    kDemand = customer_info.getDemand();//读取需求
    return kDemand.size();
}();

vector<vector<double>> edge_d = vector<vector<double>>(kNumCustomer + 1, vector<double>(kNumCustomer + 1, EDGE_K));//无人机边的权重
vector<vector<double>> edge_t = vector<vector<double>>(kNumCustomer + 1, vector<double>(kNumCustomer + 1, EDGE_K));//卡车边的权重

//计算最大无人机距离
const double kDistMax_D = []() {
    double temp = 0;
    for (size_t i = 0; i < kDist_D.size(); i++) {
        for (size_t ii = i + 1; ii < kDist_D.size(); ii++) {
            if (kDist_D[i][ii] > temp)
                temp = kDist_D[i][ii];
        }
    }
    return temp;
}();

//计算最大卡车距离
const double kDistMax_T = []() {
    double temp = 0;
    for (size_t i = 0; i < kDist_T.size(); i++) {
        for (size_t ii = i + 1; ii < kDist_T.size(); ii++) {
            if (kDist_T[i][ii] > temp)
                temp = kDist_T[i][ii];
        }
    }
    return temp;
}();

//计算最大需求量差异
const double KDemandDifMax = []() {
    double temp = 0;
    for (size_t i = 0; i < kDemand.size(); i++) {
        for (size_t ii = i + 1; ii < kDemand.size(); ii++) {
            if (abs(kDemand[i] - kDemand[ii]) > temp)
                temp = kDemand[i] - kDemand[ii];
        }
    }
    return temp;
}();

//计算总需求量
const double kDemandTotal = []() {
    double t = 0;
    for (size_t i = 0; i < kDemand.size(); i++) {
        t += kDemand[i];
    }
    return t;
}();

