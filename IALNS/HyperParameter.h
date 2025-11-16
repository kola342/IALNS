#ifndef HYPERPARAMETER_H
#define HYPERPARAMETER_H

#include <string>
#include <random>
#define M 10000000000;

extern std::mt19937 gen;


//与算法有关的
extern const double W;//
extern const double T_END;//结束温度
extern const double RATE_T;//温度下降速率
extern const int STEP_SIZE;//更新步长
extern const double NOISE;//噪声比例
extern const int REGRET_K;//regret-k
extern const int NOIMPRO;//最大未改进的次数
extern const int SEG;//算子得分的更新步长

//节省值的参数
extern const double DELTA_C;//节省值计算的cost
extern const double DELTA_DD;//节省值计算的demand和diatance

//与sigmoid相关的
extern const double ALPHA, BETA;//sigmoid函数的形状参数
extern const int NUM_REMOVAL_MAX, NUM_REMOVAL_MIN;//移除客户点的最大最小个数

//移除的随机性
extern const double COEFF_REMOVE;//y的p次方的p

//基于相关性的移除
extern const double COEFF_SHAW_DIST;//相关性的距离系数
extern const double COEFF_SHAW_DEMAND;//相关性的需求量系数
extern const double COEFF_SHAW_DRONE;//相关性的是否同一无人机路径系数
extern const double COEFF_SHAW_MOBILE;//相关性的是否同一移动卫星系数

//基于边的移除
extern const double R_EDGE;//边的信息的衰减率
extern const double EDGE_K;

//算子的更新
extern const double R;//算子得分衰减率
extern const double DELTA1;
extern const double DELTA2;
extern const double DELTA3;
extern const double DELTA4;
extern const double OPERATOR_D;

//无人机参数
extern const double cE;
extern const double sigmma_e;
extern const double e1D, e2D;
extern const double COEFF_EMISSION_D;//无人机排放成本
extern const double COEFF_DIST_D;//无人机距离成本
extern const int NUM_DRONE;//无人机的数量
extern const double SPEED_D;//无人机的飞行速度（计算时间）
extern const double LOAD_DRONE_MAX;//无人机最大载重量
extern const double E_MAX;//无人机最大电量
extern const double E_COEF_DIST;//与距离有关的电量系数
extern const double E_COEF_DIST_LOAD;//与载重距离有关的电量系数

//卡车参数
extern const double e1T, e2T, sigmma_f;
extern const double COEFF_EMISSION_T;//卡车排放成本
extern const double SPEED_T;//卡车的速度（计算时间）
extern const double COEFF_DIST_T;//卡车距离成本
extern const double COEFF_WAIT;//等待时间成本
extern const double O_COEFF_DIST;//与距离有关的油耗系数
extern const double O_COEFF_DIST_LOAD;//与载重距离有关的油耗系数




#endif
