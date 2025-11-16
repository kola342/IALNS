#include "HyperParameter.h"
#include <time.h>
//std::mt19937 gen((unsigned int)time(nullptr)); // 定义随机数生成器对象gen，使用time(nullptr)作为随机数生成器的种子

std::mt19937 gen(9);

static double B = 0.0234;
static double K = 0.25;
static double N = 38.34;
static double V = 4.5;
static double phi = 1.0 / 737;
static double mt = 3500;
static double g = 9.81;
static double cr = 0.01;
static double cd = 0.6;
static double eta = 0.45;
static double epsilon = 0.45;
static double rho = 1.204;
static double A = 6;
const double T_END = 0.0001;//结束温度

const int NUM_DRONE = 1;//无人机的数量


//与算法有关的
const double W = 0.5;//比初始解差W的解，以0.5的概率采纳
const double RATE_T=0.997;//温度下降速率
const double NOISE=0.05;//噪声比例5%
const int REGRET_K=3;//regret-k>=1,<=2
const int NOIMPRO=200;//最大未改进的次数

//节省值的参数
const double DELTA_C = 1;
const double DELTA_DD = 1;

//与sigmoid相关的
const double ALPHA = 0.05;//平缓度，越小越平缓
const double BETA = 90;//sigmoid函数的形状参数,对称点的横坐标，与n有关
const int NUM_REMOVAL_MAX = 9;//移除客户点的最大个数
const int NUM_REMOVAL_MIN = 3;//移除客户点的最小个数

//移除的随机性
const double COEFF_REMOVE=4;//y的p次方的p

//基于相关性的移除
const double COEFF_SHAW_DIST=1;//相关性的距离系数
const double COEFF_SHAW_DEMAND=1;//相关性的需求量系数
const double COEFF_SHAW_DRONE=1;//相关性的是否同一无人机路径系数
const double COEFF_SHAW_MOBILE=1;//相关性的是否同一移动卫星系数

//基于边的移除
const double R_EDGE=0.7;//边的信息的衰减率
const double EDGE_K=1;

//算子的更新
const int SEG = 10;//算子得分的更新步长
const double R = 0.1;//算子得分衰减率
const double DELTA1=15;//best
const double DELTA2=10;//由于当前解
const double DELTA3=5;//依概率接受劣解
const double DELTA4=1;//劣解
const double OPERATOR_D=1;

//无人机参数
const double cE = 0.264;
const double sigmma_e = 0.5366 / 0.93;
const double e1D = 0.7075 / 1000;
const double e2D = 0.0836 / 1000;
const double SPEED_D = 11.5;//无人机的飞行速度（计算时间）
const double COEFF_EMISSION_D = cE* sigmma_e;//无人机排放成本cE*sigmma_e
const double COEFF_DIST_D = 0.684 / 1000;//无人机距离成本cD
const double LOAD_DRONE_MAX = 4.5;//无人机最大载重量E
const double E_MAX = 1.6;//无人机最大电量Kwh
const double E_COEF_DIST = e1D/ SPEED_D;//与距离有关的电量系数e1D/v
const double E_COEF_DIST_LOAD = e2D/ SPEED_D;//与载重距离有关的电量系数e2D/v

//卡车参数
const double e1T = B * phi * (K * N * V + (mt * g * cr + 0.5 * cd * A * rho * SPEED_T * SPEED_T) * SPEED_T / 1000 / eta / epsilon);
const double e2T = B * phi * g * cr * SPEED_T / 1000 / eta / epsilon;
const double sigmma_f = 3.24;
const double SPEED_T = 11.5;//卡车速度
const double COEFF_EMISSION_T = 0.264 * 3.24 ;//卡车排放成本cE&sigma_f
const double COEFF_DIST_T = 5.693 / 1000;//卡车距离成本cT
const double COEFF_WAIT = 0.005;//等待时间成本cw
const double O_COEFF_DIST = B* phi* (K* N* V + (mt * g * cr + 0.5 * cd * A * rho * SPEED_T * SPEED_T) * SPEED_T / 1000 / eta / epsilon)/ SPEED_T;
//与距离有关的油耗系数e1T/v
const double O_COEFF_DIST_LOAD = B* phi* g* cr* SPEED_T / 1000 / eta / epsilon/ SPEED_T;//与载重距离有关的油耗系数e2T/v

