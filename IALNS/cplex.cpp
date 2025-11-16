#include "cplex.h"
#include "HyperParameter.h"
#include "GlobalVariable.h"
#include <ilcplex/ilocplex.h>
#include <vector>

static void cplex(vector<double>& d_j, vector<vector<double>>& dis, int N, vector<vector<vector<int>>>& y_ast, vector<vector<vector<double>>>& q_ast, double& t_ast, double&);
static double optimal_Route(list<Route>::iterator, vector<MobileInfo>&);
static void getDist_T(vector<vector<double>>&);
static void getDist_D(vector<vector<double>>&);
static void getDemand(vector<double>&);
void intensify(ImprovedALNS& s);
list<Satelite> memory;
static const int size_memory = 200;
using namespace std;

ILOSTLBEGIN
 
void cplex(vector<double>& d_j, vector<vector<double>>& dis, int N, vector<vector<vector<int>>>& y_ast, vector<vector<vector<double>>>& q_ast, double& t_ast,double& obj) {
    // 参数定义
    double c_D = COEFF_DIST_D;  // 卡车操作成本
    double c_e = cE;  // 与卡车能量相关的成本
    double c_w = COEFF_WAIT;
    double delta_e = sigmma_e;  // 调整因子
    double e1_D = e1D;  // 每个卡车的 e1 能量
    double e2_D = e2D;  // 每个卡车的 e2 能量
    double v_D = SPEED_D;  // 卡车的速度，假设一个值
    double E = E_MAX;  // 每辆卡车的能量限制
    double Q = LOAD_DRONE_MAX;  // 卡车的容量，假设一个值
    int K = NUM_DRONE;  // 无人机数量
    //int N = 10;  // 客户数量

    // N 个位置之间的距离矩阵（示例数据）
    //vector<vector<double>> dis(N + 1, vector<double>(N + 1, 0));
    //vector<double> d_j(N + 1, 0);  // 位置的需求


    // CPLEX 环境初始化
    IloEnv env;
    try {
        IloModel model(env);
        IloCplex cplex(model);

        // 决策变量定义
        IloArray<IloArray<IloArray<IloBoolVar>>> y(env, N + 1);
        IloArray<IloArray<IloArray<IloNumVar>>> q(env, N + 1);
        IloNumVar t(env, 0, IloInfinity);

        model.add(t);

        // 初始化 y 和 q 变量
        for (int i = 0; i <= N; i++) {
            y[i] = IloArray<IloArray<IloBoolVar>>(env, N + 1);
            q[i] = IloArray<IloArray<IloNumVar>>(env, N + 1);
            for (int j = 0; j <= N; j++) {
                y[i][j] = IloArray<IloBoolVar>(env, K);  // y[i][j] 是大小为 K 的布尔变量数组
                q[i][j] = IloArray<IloNumVar>(env, K);  // q[i][j] 是大小为 K 的浮动变量数组
                for (int d = 0; d < K; d++) {
                    y[i][j][d] = IloBoolVar(env);
                    q[i][j][d] = IloNumVar(env, 0, IloInfinity, IloNumVar::Float);  // 为 q[i][j][d] 初始化
                    model.add(q[i][j][d]);
                    model.add(y[i][j][d]);
                }
            }
        }


        // 目标函数：卡车的操作成本 + 能量相关的成本 + 目标变量 t 的权重
        IloExpr objective(env);
        for (int i = 0; i <= N; i++) {
            for (int ii = 0; ii <= N; ii++) {
                for (int dd = 0; dd < K; dd++) {
                    objective += (c_D + c_e * delta_e * e1_D / v_D) * dis[i][ii] * y[i][ii][dd];
                    objective += (c_e * delta_e * e2_D / v_D) * q[i][ii][dd] * dis[i][ii];
                }
            }
        }
        objective += t * c_w;
        model.add(IloMinimize(env, objective));

        // 约束条件

        // 1. 每个位置 j 必须有且仅有一辆卡车
        for (int j = 1; j <= N; j++) {
            IloExpr constraint(env);
            for (int i = 0; i <= N; i++) {
                for (int d = 0; d < K; d++) {
                    constraint += y[i][j][d];
                }
            }
            model.add(IloConstraint(constraint == 1));
        }

        // 2. 卡车流量对称约束
        for (int j = 0; j <= N; j++) {
            for (int d = 0; d < K; d++) {
                IloExpr constraint1(env), constraint2(env);
                for (int i = 0; i <= N; i++) {
                    constraint1 += y[i][j][d];
                    constraint2 += y[j][i][d];
                }
                model.add(IloConstraint(constraint1 == constraint2));
                model.add(IloConstraint(y[j][j][d] == 0));
            }
        }

        for (int d = 0; d < K; d++) {
            IloExpr constraint(env);
            for (int i = 1; i <= N; i++) {
                constraint += y[0][i][d];
            }
            model.add(IloConstraint(constraint <= 1));
        }

        // 3. 满足需求的约束
        for (int j = 1; j <= N; j++) {
            for (int d = 0; d < K; d++) {
                IloExpr constraint1(env), constraint2(env);
                for (int i = 0; i <= N; i++) {
                    constraint1 += q[i][j][d] - q[j][i][d];
                    constraint2 += y[i][j][d];
                }
                model.add(IloConstraint(constraint1 == d_j[j] * constraint2));
            }
        }

        // 4. 从虚拟位置 0 的数量为 0
        for (int i = 0; i <= N; i++) {
            for (int d = 0; d < K; d++) {
                model.add(IloConstraint(q[i][0][d] == 0));
            }
        }

        // 5. 卡车的容量约束
        for (int i = 0; i <= N; i++) {
            for (int j = 0; j <= N; j++) {
                for (int d = 0; d < K; d++) {
                    model.add(IloConstraint(q[i][j][d] <= Q * y[i][j][d]));
                }
            }
        }

        // 6. 能量消耗约束
        for (int d = 0; d < K; d++) {
            IloExpr energyConstraint(env);
            for (int i = 0; i <= N; i++) {
                for (int j = 0; j <= N; j++) {
                    energyConstraint += (e1_D / v_D) * y[i][j][d] * dis[i][j];
                    energyConstraint += (e2_D / v_D) * q[i][j][d] * dis[i][j];
                }
            }
            model.add(IloConstraint(energyConstraint <= E));
        }

        // 7. 时间约束
        for (int d = 0; d < K; d++) {
            IloExpr timeConstraint(env);
            for (int i = 0; i <= N; i++) {
                for (int j = 0; j <= N; j++) {
                    timeConstraint += y[i][j][d] * dis[i][j];
                }
            }
            model.add(IloConstraint(t >= timeConstraint / v_D));
        }

        std::ofstream nullStream;
        nullStream.open("NUL");     // Windows
        cplex.setOut(nullStream);  // 设置输出流为空，从而禁止打印

        // 求解模型
        cplex.extract(model);

        // 创建一个 IloNumVarArray 来存储所有决策变量
        IloNumVarArray varArray(env);
        IloNumArray startValues(env);

        // 填充变量数组和起始值数组
        for (int i = 0; i <= N; i++) {
            for (int j = 0; j <= N; j++) {
                for (int d = 0; d < K; d++) {
                    varArray.add(y[i][j][d]);
                    startValues.add(y_ast[i][j][d]);  // 给变量 y[i][j][d] 设置初始值
                    varArray.add(q[i][j][d]);
                    startValues.add(q_ast[i][j][d]);  // 给变量 q[i][j][d] 设置初始值
                }
            }
        }
        varArray.add(t);
        startValues.add(t_ast);

        // 向 CPLEX 设置 MIP 启动
        cplex.addMIPStart(varArray, startValues);

        if (cplex.solve()) {
            for (int i = 0; i <= N; i++) {
                for (int j = 0; j <= N; j++) {
                    for (int d = 0; d < K; d++) {
                        y_ast[i][j][d] = static_cast<int>(round(cplex.getValue(y[i][j][d])));
                        q_ast[i][j][d] = cplex.getValue(q[i][j][d]);
                    }
                }
            }
            t_ast = cplex.getValue(t);
            obj = cplex.getObjValue();
        }
        else {
            cout << "Solution not found!" << endl;
        }

        nullStream.close();

    }
    catch (IloException& e) {
        cerr << "CPLEX Exception: " << e.getMessage() << endl;
    }
    catch (...) {
        cerr << "Unknown exception caught!" << endl;
    }

    env.end();
}

void warmStart(ImprovedALNS& s, int time) {
    vector<vector<vector<vector<bool>>>> y_ast;
    vector<vector<bool>> x_ast;
    vector<bool> w_ast;
    vector<double> t_ast;
    vector<vector<vector<vector<double>>>> qD_ast;
    vector<vector<double>> qT_ast;
    s.outputSolution(y_ast, x_ast, w_ast, t_ast, qD_ast, qT_ast);
    vector<vector<double>> dis_D, dis_T;
    vector<double> demand;
    getDist_D(dis_D);
    getDist_T(dis_T);
    getDemand(demand);

    // 创建 CPLEX 环境
    IloEnv env;
    try {
        // 输入数据
        double Qt = 1000;
        int c1_n = 0; // TDC数量
        for (int i = 0; i < kNumCustomer; i++) {
            if (kDemand[i] <= LOAD_DRONE_MAX)
                c1_n++;
        }
        int c2_n = kNumCustomer-c1_n; // TC数量
        int D_n = NUM_DRONE;  // 无人机数量
        double vd = SPEED_D; // 无人机的速度
        double vt = SPEED_T; // 卡车速度
        double Qd = LOAD_DRONE_MAX; // 无人机最大载重
        double cw = COEFF_WAIT; // 卡车每单位等待时间的成本

        double E = E_MAX;  // 无人机最大电量
        double cD = COEFF_DIST_D; // 无人机的旅行成本
        double cT = COEFF_DIST_T; // 卡车的旅行成本
        double M_i = 1000;
        


        // 创建模型
        IloModel model(env);
        // 创建求解器
        IloCplex cplex(model);

        // 定义决策变量
        // C++ 中变量声明
        IloArray<IloBoolVarArray> x(env, c1_n + c2_n + 1);
        IloArray<IloArray<IloArray<IloBoolVarArray>>> y(env, c1_n + c2_n+1);
        IloBoolVarArray w(env, c1_n + c2_n+1);
        IloArray<IloArray<IloArray<IloNumVarArray>>> qD(env, c1_n + c2_n+1);
        IloArray<IloNumVarArray> qT(env, c1_n + c2_n + 1);
        IloNumVarArray t(env, c1_n + c2_n + 1, 0, INFINITY, IloNumVar::Float);

        for (int i = 0; i <= c1_n + c2_n; i++) {
            x[i] = IloBoolVarArray(env, c1_n + c2_n + 1);
            qT[i] = IloNumVarArray(env, c1_n + c2_n + 1, 0, Qt, IloNumVar::Float);
            for (int j = 0; j <= c1_n + c2_n; j++) {
                model.add(x[i][j]);
                model.add(qT[i][j]);
            }
        }
        for (int i = 1; i <= c1_n + c2_n; i++) {
            y[i] = IloArray<IloArray<IloBoolVarArray>>(env, c1_n + c2_n + 1);
            qD[i] = IloArray<IloArray<IloNumVarArray>>(env, c1_n + c2_n + 1);
            model.add(w[i]);
            model.add(t[i]);
            for (int j = 1; j <= c1_n + c2_n; j++) {
                y[i][j] = IloArray<IloBoolVarArray>(env, D_n);
                qD[i][j] = IloArray<IloNumVarArray>(env, D_n);
                for (int d = 0; d < D_n; d++) {
                    y[i][j][d] = IloBoolVarArray(env, c1_n + c2_n + 1);
                    qD[i][j][d] = IloNumVarArray(env, c1_n + c2_n + 1, 0, Qd, IloNumVar::Float);
                    for (int v = 1; v <= c1_n + c2_n; v++) {
                        model.add(y[i][j][d][v]);
                        model.add(qD[i][j][d][v]);
                    }
                }
            }
        }

        // 定义目标函数
        IloExpr objective(env);
        // 无人机旅行成本
        for (int i = 1; i <= c1_n + c2_n; ++i) {
            for (int j = 1; j <= c1_n + c2_n; ++j) {
                for (int d = 0; d < D_n; ++d) {
                    for (int v = 1; v <= c1_n + c2_n; ++v) {
                        objective += cD * y[i][j][d][v] * dis_D[i][j]; // 无人机旅行成本
                    }
                }
            }
        }

        // 卡车旅行成本
        for (int i = 0; i <= c1_n + c2_n; ++i) {
            for (int j = 0; j <= c1_n + c2_n; ++j) {
                objective += cT * x[i][j] * dis_T[i][j];
            }
        }

        // 无人机排放成本
        for (int i = 1; i <= c1_n + c2_n; ++i) {
            for (int j = 1; j <= c1_n + c2_n; ++j) {
                for (int d = 0; d < D_n; ++d) {
                    for (int v = 1; v <= c1_n + c2_n; ++v) {
                        objective += cE * sigmma_e * (e1D * y[i][j][d][v] * dis_D[i][j] / vd + e2D * qD[i][j][d][v] * dis_D[i][j] / vd);
                    }
                }
            }
        }

        // 卡车排放成本
        for (int i = 0; i <= c1_n + c2_n; ++i) {
            for (int j = 0; j <= c1_n + c2_n; ++j) {
                objective += cE * sigmma_f * (e1T * x[i][j] * dis_T[i][j] / vt + e2T * qT[i][j] * dis_T[i][j] / vt);
            }
        }

        // 卡车等待成本
        for (int v = 1; v <= c1_n + c2_n; ++v) {
            objective += cw * t[v];
        }

        model.add(IloMinimize(env, objective));

        // 约束1: forall(j in c) sum(i in c0) x[i][j] == w[j];
        for (int j = 1; j <= c1_n + c2_n; ++j) {
            IloExpr constraint1(env);
            for (int i = 0; i <= c1_n + c2_n; ++i) {
                constraint1 += x[i][j];
            }
            model.add(constraint1 == w[j]);
        }

        // 约束2: forall(i in c2) w[i] == 1;
        for (int i = c1_n+1; i <= c1_n + c2_n; ++i) {
            model.add(w[i] == 1);
        }

        // 约束3: forall(i in c0) x[i][i] == 0;
        for (int i = 0; i <= c1_n + c2_n; ++i) {
            model.add(x[i][i] == 0);
        }

        // 约束4: forall(i in c, d in D, j in c) y[i][i][d][j] == 0;
        for (int i = 1; i <= c1_n + c2_n; ++i) {
            for (int d = 0; d < D_n; ++d) {
                for (int j = 1; j <= c1_n + c2_n; ++j) {
                    model.add(y[i][i][d][j] == 0);
                }
            }
        }

        // 约束5: forall(j in c) sum(i in c, d in D, v in c : v!=j) y[i][j][d][v]==1-w[j]
        for (int j = 1; j <= c1_n + c2_n; ++j) {
            IloExpr constraint5(env);
            for (int i = 1; i <= c1_n + c2_n; ++i) {
                for (int d = 0; d < D_n; ++d) {
                    for (int v = 1; v <= c1_n + c2_n; ++v) {
                        if(v!=j)
                            constraint5 += y[i][j][d][v];
                    }
                }
            }
            model.add(constraint5 == 1 - w[j]);
        }

        // 约束6: forall(i in c) sum(s in c, t in c, d in D) y[s][t][d][i] + M * w[i] >= 0;
        for (int i = 1; i <= c1_n + c2_n; ++i) {
            IloExpr constraint6(env);
            for (int s = 1; s <= c1_n + c2_n; ++s) {
                for (int t = 1; t <= c1_n + c2_n; ++t) {
                    for (int d = 0; d < D_n; ++d) {
                        constraint6 += y[s][t][d][i];
                    }
                }
            }
            model.add(constraint6 + M_i * w[i] >= 0);
        }

        // 约束7: forall(i in c) sum(s in c, t in c, d in D) y[s][t][d][i] - M * w[i] <= 0;
        for (int i = 1; i <= c1_n + c2_n; ++i) {
            IloExpr constraint7(env);
            for (int s = 1; s <= c1_n + c2_n; ++s) {
                for (int t = 1; t <= c1_n + c2_n; ++t) {
                    for (int d = 0; d < D_n; ++d) {
                        constraint7 += y[s][t][d][i];
                    }
                }
            }
            model.add(constraint7 - M_i * w[i] <= 0);
        }

        // 约束8: sum(j in c) x[0][j] == 1;
        IloExpr constraint8(env);
        for (int j = 1; j <= c1_n + c2_n; ++j) {
            constraint8 += x[0][j];
        }
        model.add(constraint8 == 1);

        // 约束9: sum(j in c) x[j][0] == 1;
        IloExpr constraint9(env);
        for (int j = 1; j <= c1_n + c2_n; ++j) {
            constraint9 += x[j][0];
        }
        model.add(constraint9 == 1);

        // 约束10: forall(i in c) sum(j in c0) x[i][j] == sum(j in c0) x[j][i];
        for (int i = 1; i <= c1_n + c2_n; ++i) {
            IloExpr constraint10(env);
            IloExpr constraint10_rev(env);
            for (int j = 0; j <= c1_n + c2_n; ++j) {
                constraint10 += x[i][j];
                constraint10_rev += x[j][i];
            }
            model.add(constraint10 == constraint10_rev);
        }

        // 约束11: forall(i in c) sum(j in c0 : j != i) qT[j][i] - sum(j in c0 : j != i) qT[i][j] + M * (1 - w[i]) >= demand[i] + sum(t in c1 : t != i, s in c, d in D) (demand[t] * y[s][t][d][i]);
        for (int i = 1; i <= c1_n + c2_n; ++i) {
            IloExpr constraint11(env);
            IloExpr constraint11_rhs(env);
            for (int j = 0; j <= c1_n + c2_n; ++j) {
                if (i != j) {
                    constraint11 += qT[j][i];
                    constraint11_rhs += qT[i][j];
                }
            }
            for (int t = 1; t <= c1_n; ++t) {
                if (t != i) {
                    for (int s = 1; s <= c1_n + c2_n; ++s) {
                        for (int d = 0; d < D_n; ++d) {
                            constraint11_rhs += demand[t] * y[s][t][d][i];
                        }
                    }
                }
            }
            model.add(constraint11 - constraint11_rhs + M_i * (1 - w[i]) >= demand[i]);
        }

        // 约束12: forall(i in c) sum(j in c0 : j != i) qT[j][i] - sum(j in c0 : j != i) qT[i][j] - M * (1 - w[i]) <= demand[i] + sum(t in c1 : t != i, s in c, d in D) (demand[t] * y[s][t][d][i]);
        for (int i = 1; i <= c1_n + c2_n; ++i) {
            IloExpr constraint12(env);
            IloExpr constraint12_rhs(env);
            for (int j = 0; j <= c1_n + c2_n; ++j) {
                if (i != j) {
                    constraint12 += qT[j][i];
                    constraint12_rhs += qT[i][j];
                }
            }
            for (int t = 1; t <= c1_n; ++t) {
                if (t != i) {
                    for (int s = 1; s <= c1_n + c2_n; ++s) {
                        for (int d = 0; d < D_n; ++d) {
                            constraint12_rhs += demand[t] * y[s][t][d][i];
                        }
                    }
                }
            }
            model.add(constraint12 - constraint12_rhs - M_i * (1 - w[i]) <= demand[i]);
        }

        // 约束13: forall(i in c) qT[i][0] == 0;
        for (int i = 1; i <= c1_n + c2_n; ++i) {
            model.add(qT[i][0] == 0);
        }

        // 约束14: sum(i in c) qT[0][i] == sum(j in c) demand[j];
        IloExpr constraint14(env);
        for (int i = 1; i <= c1_n + c2_n; ++i) {
            constraint14 += qT[0][i];
        }
        IloExpr demand_sum(env);
        for (int j = 1; j <= c1_n + c2_n; ++j) {
            demand_sum += demand[j];
        }
        model.add(constraint14 == demand_sum);

        // 约束15: forall(i in c0, j in c0) qT[i][j] <= Qt * x[i][j];
        for (int i = 0; i <= c1_n + c2_n; ++i) {
            for (int j = 0; j <= c1_n + c2_n; ++j) {
                model.add(qT[i][j] <= Qt * x[i][j]);
            }
        }

        // 约束16: forall(d in D, v in c) e1D * sum(i in c, j in c) (y[i][j][d][v] * dis_D[i][j] / vd) + e2D * sum(i in c, j in c) (qD[i][j][d][v] * dis_D[i][j] / vd) <= E;
        for (int d = 0; d < D_n; ++d) {
            for (int v = 1; v <= c1_n + c2_n; ++v) {
                IloExpr constraint16(env);
                for (int i = 1; i <= c1_n + c2_n; ++i) {
                    for (int j = 1; j <= c1_n + c2_n; ++j) {
                        constraint16 += y[i][j][d][v] * dis_D[i][j];
                        constraint16 += qD[i][j][d][v] * dis_D[i][j];
                    }
                }
                model.add(e1D * constraint16 / vd + e2D * constraint16 / vd <= E);
            }
        }

        // 约束17: forall(i in c, d in D) sum(j in c1) y[i][j][d][i]<=w[i]
        for (int i = 1; i <= c1_n + c2_n; ++i) {
            for (int d = 0; d < D_n; ++d) {
                IloExpr constraint17(env);
                for (int j = 1; j <= c1_n; ++j) {
                    constraint17 += y[i][j][d][i];
                }
                model.add(constraint17 <= w[i]);
            }
        }


        // 约束19: forall(i in c, d in D, v in c) sum(j in c) y[i][j][d][v] == sum(j in c) y[j][i][d][v];
        for (int i = 1; i <= c1_n+c2_n; ++i) {
            for (int d = 0; d < D_n; ++d) {
                for (int v = 1; v <= c1_n + c2_n; ++v) {
                    IloExpr constraint19(env);
                    for (int j = 1; j <= c1_n + c2_n; ++j) {
                        constraint19 += y[i][j][d][v];
                        constraint19 -= y[j][i][d][v];
                    }
                    model.add(constraint19 == 0);
                }
            }
        }

        // 约束20: forall(i in c1, d in D, v in c) sum(j in c : j != i) qD[j][i][d][v] - sum(j in c : j != i) qD[i][j][d][v] + M * w[i] >= demand[i] * sum(j in c : j != i) y[i][j][d][v];
        for (int i = 1; i <= c1_n; ++i) {
            for (int d = 0; d < D_n; ++d) {
                for (int v = 1; v <= c1_n + c2_n; ++v) {
                    IloExpr constraint20(env);
                    IloExpr constraint20_rhs(env);
                    IloExpr constraint20_1(env);
                    for (int j = 1; j <= c1_n + c2_n; ++j) {
                        if (i != j) {
                            constraint20 += qD[j][i][d][v];
                            constraint20_rhs += qD[i][j][d][v];
                            constraint20_1 += y[i][j][d][v];
                        }
                    }
                    model.add(constraint20 - constraint20_rhs + M_i * w[i] >= demand[i] * constraint20_1);
                }
            }
        }

        // 约束21: forall(i in c1, d in D, v in c) sum(j in c : j != i) qD[j][i][d][v] - sum(j in c : j != i) qD[i][j][d][v] - M * w[i] <= demand[i] * sum(j in c : j != i) y[i][j][d][v];
        for (int i = 1; i <= c1_n; ++i) {
            for (int d = 0; d < D_n; ++d) {
                for (int v = 1; v <= c1_n + c2_n; ++v) {
                    IloExpr constraint21(env);
                    IloExpr constraint21_rhs(env);
                    IloExpr constraint21_1(env);
                    for (int j = 1; j <= c1_n + c2_n; ++j) {
                        if (i != j) {
                            constraint21 += qD[j][i][d][v];
                            constraint21_rhs += qD[i][j][d][v];
                            constraint21_1 += y[i][j][d][v];
                        }
                    }
                    model.add(constraint21 - constraint21_rhs - M_i * w[i] <= demand[i] * constraint21_1);
                }
            }
        }

        // 约束22: forall(i in c,j in c, d in D) qD[j][i][d][i] == 0;
        for (int i = 1; i <= c1_n + c2_n; ++i) {
            for (int j = 1; j <= c1_n + c2_n; ++j) {
                for (int d = 0; d < D_n; ++d) {
                    model.add(qD[j][i][d][i] == 0);
                }
            }
        }

        // 约束23: forall(i in c,j in c1, d in D) qD[i][j][d][i] <= Qd;
        for (int i = 1; i <= c1_n + c2_n; ++i) {
            for (int j = 1; j <= c1_n; ++j) {
                for (int d = 0; d < D_n; ++d) {
                    model.add(qD[i][j][d][i] <= Qd);
                }
            }
        }

        // 约束24: forall(i in c, j in c, d in D, v in c) qD[i][j][d][v] <= Qd * y[i][j][d][v];
        for (int i = 1; i <= c1_n + c2_n; ++i) {
            for (int j = 1; j <= c1_n + c2_n; ++j) {
                for (int d = 0; d < D_n; ++d) {
                    for (int v = 1; v <= c1_n + c2_n; ++v) {
                        model.add(qD[i][j][d][v] <= Qd * y[i][j][d][v]);
                    }
                }
            }
        }

        // 约束25: forall(v in c, d in D) t[v] >= sum(i in c, j in c)(y[i][j][d][v] * dis_D[i][j] / vd);
        for (int v = 1; v <= c1_n + c2_n; ++v) {
            for (int d = 0; d < D_n; ++d) {
                IloExpr constraint25(env);
                for (int i = 1; i <= c1_n + c2_n; ++i) {
                    for (int j = 1; j <= c1_n + c2_n; ++j) {
                        constraint25 += y[i][j][d][v] * dis_D[i][j];
                    }
                }
                model.add(t[v] >= constraint25 / vd);
            }
        }

        // 求解模型
        cplex.extract(model);

        // 创建一个 IloNumVarArray 来存储所有决策变量
        IloNumVarArray varArray(env);
        IloNumArray startValues(env);


        for (int i = 0; i <= c1_n + c2_n; i++) {
            for (int j = 0; j <= c1_n + c2_n; j++) {
                varArray.add(x[i][j]);
                startValues.add(x_ast[i][j]);
                varArray.add(qT[i][j]);
                startValues.add(qT_ast[i][j]);
            }
        }
        for (int i = 1; i <= c1_n + c2_n; i++) {
            varArray.add(w[i]);
            startValues.add(w_ast[i]);
            varArray.add(t[i]);
            startValues.add(t_ast[i]);
            for (int j = 1; j <= c1_n + c2_n; j++) {
                for (int d = 0; d < D_n; d++) {
                    for (int v = 1; v <= c1_n + c2_n; v++) {
                        varArray.add(y[i][j][d][v]);
                        startValues.add(y_ast[i][j][d][v]);
                        varArray.add(qD[i][j][d][v]);
                        startValues.add(qD_ast[i][j][d][v]);
                    }
                }
            }
        }

        // 向 CPLEX 设置 MIP 启动
        cplex.addMIPStart(varArray, startValues);


        // 设置求解时间限制
        cplex.setParam(IloCplex::TiLim, time); // 设置最大求解时间为 7200 秒

        // 求解模型
        if (cplex.solve()) {
            // 打印结果
            std::cout << "Solution status: " << cplex.getStatus() << std::endl;
            std::cout << "Objective value: " << cplex.getObjValue() << std::endl;
            for (int i = 0; i <= c1_n + c2_n; i++) {
                for (int j = 0; j <= c1_n + c2_n; j++) {
                    if (static_cast<int>(round(cplex.getValue(x[i][j]))) != 0) {
                        cout << "x[" << i << "," << j << "]=" << static_cast<int>(round(cplex.getValue(x[i][j]))) << endl;
                        cout << "qT[" << i << "," << j << "]=" << cplex.getValue(qT[i][j]) << endl;
                    }
                }
            }
            for (int i = 1; i <= c1_n + c2_n; i++) {
                if(static_cast<int>(round(cplex.getValue(w[i]))) != 0)
                    cout << "w[" << i << "]=" << static_cast<int>(round(cplex.getValue(w[i]))) << endl;
                if(cplex.getValue(t[i]) != 0)
                    cout << "t[" << i << "]=" << cplex.getValue(t[i]) << endl;
                for (int j = 1; j <= c1_n + c2_n; j++) {
                    for (int d = 0; d < D_n; d++) {
                        for (int v = 1; v <= c1_n + c2_n; v++) {
                            if (static_cast<int>(round(cplex.getValue(y[i][j][d][v]))) != 0) {
                                cout << "y[" << i << "," << j << "," << d << "," << v << "]=" << static_cast<int>(round(cplex.getValue(y[i][j][d][v]))) << endl;
                                cout << "qD[" << i << "," << j << "," << d << "," << v << "]=" << cplex.getValue(qD[i][j][d][v]) << endl;
                            }
                        }
                    }
                }
            }
            // 其他变量的输出
        }
        else {
            std::cout << "No solution found!" << std::endl;
        }
    }
    catch (IloException& e) {
        std::cerr << "Error: " << e.getMessage() << std::endl;
    }
    catch (...) {
        std::cerr << "Unknown error" << std::endl;
    }

    // 释放 CPLEX 环境
    env.end();

}

void intensify(ImprovedALNS& s) {//结束后计算cost，前向距离和后向距离,mobile
    list<Route>& solution = s.getSolution();//获得解
    vector<MobileInfo>& mobile = s.getMobile();//获得移动卫星
    for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {
        if (mobile[it1->id].num > 1) {//是移动卫星
            mobile[it1->id].cust.sort();//cust从小到达排序
            bool find =false;//是否在memory中找到
            list<Satelite>::iterator it_memory;//存储memory的位置
            for (auto it = memory.begin(); it != memory.end(); it++) {//遍历memory
                if (it1->id == it->satelite.id && mobile[it1->id].num == it->num) {
                    auto itt1 = it->cust.begin();
                    bool same = true;//假定相同
                    for (auto itt = mobile[it1->id].cust.begin(); itt != mobile[it1->id].cust.end(); itt++) {//逐一检测
                        if (*itt != *itt1) {//不同的情况
                            same = false;
                            break;
                        }
                        itt1++;
                    }
                    if (same) {//判断是否相同
                        find = true;//在memory中找到
                        it_memory = it;//记录在memory的指针
                        break;
                    }
                }
            }
            if (find) {//若找到
                *it1 = it_memory->satelite;
                double temp_cost = mobile[it1->id].cost;
                mobile[it1->id].cost = it_memory->cost;
                //将其移向最后
                memory.insert(memory.end(), it_memory, next(it_memory));
                memory.erase(it_memory);
                s.update_less(it1, temp_cost);//只更新解和前后向
            }
            else {//没找到
                double temp_cost = mobile[it1->id].cost;
                mobile[it1->id].cost = optimal_Route(it1, mobile);//cplex求解并计算移动卫星的总成本
                s.update(it1, temp_cost);//需要更改time_second,每条路径上的电量，时间，前向后向，解
                memory.push_back(Satelite(*it1, mobile[it1->id].num, mobile[it1->id].cust, mobile[it1->id].cost));//存入memory
                if (memory.size() > size_memory) {//动态管理大小
                    memory.pop_front();
                }
            }
        }
    }
}

double optimal_Route(list<Route>::iterator it1, vector<MobileInfo>& mobile) {
    //距离、需求量、客户数量
    vector<double> demand = { 0 };//需求矩阵
    vector<vector<double>> dist;//距离矩阵
    int num = mobile[it1->id].num - 1;//客户数量
    auto temp = mobile[it1->id].cust;//存放客户集合
    list<int>::iterator temp_it;//定位移动卫星点
    for (auto it = temp.begin(); it != temp.end(); it++) {
        if (*it == it1->id) {
            temp_it = it;
            continue;
        }
    }
    //将移动卫星移向前面
    temp.erase(temp_it);
    temp.push_front(it1->id);
    vector<int> temp_index;//记录客户及其索引，temp_index[0]对应移动卫星，用于cplex
    for (auto it = temp.begin(); it != temp.end(); it++) {
        vector<double> aa;
        for (auto itt = temp.begin(); itt != temp.end(); itt++) {
            aa.push_back(kDist_D[*it][*itt]);
        }
        dist.push_back(aa);
        if(it!=temp.begin())
            demand.push_back(kDemand[*it]);
        temp_index.push_back(*it);
    }

    //初始解
    vector<vector<vector<int>>> y_ast = vector<vector<vector<int>>>(num + 1, vector<vector<int>>(num + 1, vector<int>(NUM_DRONE, 0)));
    vector<vector<vector<double>>> q_ast = vector<vector<vector<double>>>(num + 1, vector<vector<double>>(num + 1, vector<double>(NUM_DRONE, 0)));
    double t_ast = it1->time_wait;
    double obj = -1;
    vector<int> index = vector<int>(kNumCustomer, -1);//记录客户在temp_index中的索引
    int i = 0;
    for (auto it = temp.begin(); it != temp.end(); it++) {
        index[*it] = i;
        i++;
    }
    i = 0;
    for (auto it = it1->routes.begin(); it != it1->routes.end(); it++) {
        double d = it->total_demand;
        for (auto itt = next(it->route.begin()); itt != it->route.end(); itt++) {
            y_ast[index[*prev(itt)]][index[*itt]][i] = 1;
            q_ast[index[*prev(itt)]][index[*itt]][i] = d;
            d -= kDemand[*itt];
        }
        y_ast[index[*prev(it->route.end())]][index[it->route.front()]][i] = 1;
        i++;
    }
    
    //求解最优值
    cplex(demand, dist, num, y_ast, q_ast, t_ast, obj);

    //重构当前移动卫星
    list<DroneRoute> new_routes;
    for (int d = 0; d < NUM_DRONE; d++) {
        int row = 0;
        list<int> new_route;
        double temp_d = 0;
        do {
            for (int col = 0; col <= num; col++) {
                if (y_ast[row][col][d] == 1) {
                    if (row == 0) {
                        temp_d = q_ast[0][col][d];
                        new_route.push_back(it1->id);
                    }
                    if (col != 0) {
                        new_route.push_back(temp_index[col]);
                    }
                    row = col;
                    break;
                }
            }
        } while (row != 0);
        if (!new_route.empty()) {
            DroneRoute new_drone(new_route);
            new_drone.total_demand = temp_d;
            new_routes.push_back(new_drone);
        }
    }
    it1->routes = new_routes;
    it1->time_wait = t_ast;//需要更改time_second,每条路径上的电量，时间，前向后向。
    return obj;
}

void intensifyTwo(int n, ImprovedALNS& s_new, ImprovedALNS& s_current) {
    if (n!=0) {
        cout << s_new.getCost();
        intensify(s_new);
        cout << "改进" << s_new.getCost()<<endl;
    }
}

void getDist_T(vector<vector<double>>& disT) {
    disT = vector<vector<double>>(kDist_T.size(), vector<double>(kDist_T.size(), 0));

    for (size_t j = 0; j < kDist_T.size() - 1; j++) {
        disT[0][j + 1] = kDist_T[kNumCustomer][j];
    }

    for (size_t i = 0; i < kDist_T.size() - 1; i++) {
        disT[i + 1][0] = kDist_T[i][kNumCustomer];
        for (size_t j = 0; j < kDist_T.size() - 1; j++) {
            disT[i + 1][j + 1] = kDist_T[i][j];
        }
    }
}

void getDist_D(vector<vector<double>>& disD) {
    disD = vector<vector<double>>(kDist_D.size(), vector<double>(kDist_D.size(), 0));

    for (size_t j = 0; j < kDist_D.size() - 1; j++) {
        disD[0][j + 1] = kDist_D[kNumCustomer][j];
    }

    for (size_t i = 0; i < kDist_D.size() - 1; i++) {
        disD[i + 1][0] = kDist_D[i][kNumCustomer];
        for (size_t j = 0; j < kDist_D.size() - 1; j++) {
            disD[i + 1][j + 1] = kDist_D[i][j];
        }
    }
}

void getDemand(vector<double>& d) {
    d = vector<double>(kDist_T.size(), 0);
    // 遍历矩阵并写入每一行
    for (size_t i = 0; i < kDemand.size(); ++i) {
        d[i + 1] = kDemand[i];
    }
}
