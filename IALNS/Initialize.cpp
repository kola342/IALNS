#include "Initialize.h"
#include "GlobalVariable.h"
#include "HyperParameter.h"
#include "CustomerInfo.h"
#include "Route.h"
#include <iterator>

//簇结构体，包括簇的中心，簇成员
struct Cluster 
{
    vector<int> member_id = {};  //簇成员
    int center_member_id = -3;   //簇中心
};

//簇间信息（簇间距离（合并簇后，簇中心到配送一个客户所耗最大电量）、合并后的中心）
struct MergeClusInfo 
{
	double distance; //簇距离
	int center_number_id;   //簇中心
	MergeClusInfo(double a, int b) :distance(a), center_number_id(b) {}
};

//声明静态函数
static vector<Cluster> clustering();
static MergeClusInfo eConsumed(const Cluster&, const Cluster&);
static void enumerateCluster(const Cluster& a1, const Cluster& a2, MergeClusInfo&, double);//用于遍历计算a1各元素作簇中心的情况
static vector<int> routeDrone(list<Route>& solution, const Cluster& a);   //构建簇a的无人机路线
static bool improvedCNW(list<DroneRoute>& solution);    //无人机路线的C&W算法
static bool improvedCNW(list<list<Route>>& solution);    //卡车路线的C&W算法
static double save(const DroneRoute& Route1, const DroneRoute& route2, int row, int col, vector<vector<double>>& cost, vector<vector<double>>& dem, vector<vector<double>>& dis, vector<vector<double>>& e_saving);  //计算两条无人机路线的节省值
static double save(const list<Route>& route1, const list<Route>& route2, int row, int col, vector<vector<double>>& cost, vector<vector<double>>& dem, vector<vector<double>>& dis);    //计算卡车的节省值
static double calculateAveCost(DroneRoute&);
static bool lessDroneRoute(DroneRoute& a, DroneRoute& b);

static vector<double> k_demand(kNumCustomer, 0); //记录广义需求量

//构造初始解
list<Route> initialSolution() {
	list<Route> solution;   //寄存移动卫星的信息
    vector<int> customer_id_truck;  //卡车服务的客户(不包含移动卫星)
    list<list<Route>> route_truck;    //卡车路线的构建
    vector<Cluster> clusters = clustering();    //聚类操作
    for (auto& cluster_single : clusters) {
        vector<int> temp = routeDrone(solution, cluster_single);    //构建无人机路线
        customer_id_truck.insert(customer_id_truck.end(), temp.begin(), temp.end());    //合并未服务的客户集合
    }

    //计算广义需求量
    for (auto& center : solution) {
        for (auto& drone : center.routes) {
            for (auto it = drone.route.begin(); it != drone.route.end(); it++) {
                if (it != drone.route.begin()) 
                    k_demand[center.id] += kDemand[*it];
            }
        }
    }

    //构建初始卡车路线
    for (auto& satellite : solution) {
        route_truck.push_back(list<Route>(1, satellite));   //卡车路线移入移动卫星点
    }
    for (auto& customer : customer_id_truck) {
        route_truck.push_back(list<Route>(1, Route(customer))); //卡车路线移入非移动卫星点
    }
    while (route_truck.size() > 1) {
        improvedCNW(route_truck);
    }

	return route_truck.front(); //返回最终合并后的卡车路线
}

//定义静态函数

//分类算法
vector<Cluster> clustering() {  
    int n = kNumCustomer;

    // 初始化，每个点是一个单独的簇
    vector<Cluster> clusters(n);    //簇集合
    for (int i = 0; i < n; ++i) {
        clusters[i].member_id = { i };  //簇包含的成员
        clusters[i].center_member_id = i;   //簇中心
    }
    double min_clusters_dist = M;    //  定义所有簇之间的最小距离
    int min_clusters_id[2] = { 0 };    //定义拥有簇之间的最小距离的两个簇及簇中心

    // 计算初始距离矩阵
    vector<vector<MergeClusInfo>> mat_cluster_dist(n, vector<MergeClusInfo>(n, MergeClusInfo(0, 0)));   //存储簇间的信息
    for (size_t i = 0; i < clusters.size(); ++i) {
        mat_cluster_dist[i][i].distance = M;    //簇自身之间的距离设为无人机的电池容量，以防止自身与自身的合并
        for (size_t ii = i + 1; ii < clusters.size(); ++ii) {
            MergeClusInfo temp = eConsumed(clusters[i], clusters[ii]);  //计算簇间信息
            mat_cluster_dist[i][ii] = temp;
            if (mat_cluster_dist[i][ii].distance < min_clusters_dist) {
                min_clusters_dist = mat_cluster_dist[i][ii].distance;    //更新最小距离
                min_clusters_id[0] = i; //更新可合并簇的id
                min_clusters_id[1] = ii;    //更新可合并簇的id
            }
            mat_cluster_dist[ii][i] = mat_cluster_dist[i][ii];
        }
    }

    // 不断合并簇直到不能合并
    while (min_clusters_dist <= E_MAX) {
        // 合并簇
        clusters[min_clusters_id[0]].member_id.insert(clusters[min_clusters_id[0]].member_id.end(), clusters[min_clusters_id[1]].member_id.begin(), clusters[min_clusters_id[1]].member_id.end());
        clusters[min_clusters_id[0]].center_member_id = mat_cluster_dist[min_clusters_id[0]][min_clusters_id[1]].center_number_id;

        // 更新距离矩阵，重新计算合并后簇与其他簇的距离
        for (size_t i = 0; i < clusters.size(); ++i) {
            if (i != min_clusters_id[0] && i != min_clusters_id[1]) {
                MergeClusInfo temp = eConsumed(clusters[min_clusters_id[0]], clusters[i]);
                mat_cluster_dist[min_clusters_id[0]][i] = temp;
                mat_cluster_dist[i][min_clusters_id[0]] = temp;
            }
        }

        //删除被合并的簇的距离信息
        clusters.erase(clusters.begin() + min_clusters_id[1]);  //删除簇
        mat_cluster_dist.erase(mat_cluster_dist.begin() + min_clusters_id[1]);
        for (auto& row : mat_cluster_dist) {
            row.erase(row.begin() + min_clusters_id[1]);
        }

        //计算最小距离和簇中心
        min_clusters_dist = M;
        for (size_t i = 0; i < mat_cluster_dist.size(); i++) {
            for (size_t ii = i + 1; ii < mat_cluster_dist.size(); ii++) {
                if (mat_cluster_dist[i][ii].distance < min_clusters_dist) {
                    min_clusters_dist = mat_cluster_dist[i][ii].distance;   //更新最小距离
                    min_clusters_id[0] = i; //更新可合并簇的id
                    min_clusters_id[1] = ii;    //更新可合并簇的id
                }
            }
        }
    }
    return clusters;
}


//用于遍历计算a1各元素作簇中心的情况
void enumerateCluster(const Cluster& cluster1, const Cluster& cluster2, MergeClusInfo& result, double min_all_dist) {
    for (size_t i = 0; i < cluster1.member_id.size(); i++) {   //假设i为簇中心
        double sum = 0;  //记录从i出发到所有簇成员，消耗的总电量
        double temp_dist = 0;    //记录从i出发配送一个客户时，最大耗电量
        int num = 0;    //计算无人机可送达的客户数
        for (size_t ii = 0; ii < cluster1.member_id.size(); ii++) {    //配送客户为ii
            if (ii != i) {
                if (kDemand[cluster1.member_id[ii]] <= LOAD_DRONE_MAX) { //需求量小于无人机最大载重量时
                    num++;
                    double e = E_COEF_DIST * 2 * kDist_D[cluster1.member_id[i]][cluster1.member_id[ii]]
                        + E_COEF_DIST_LOAD * kDist_D[cluster1.member_id[i]][cluster1.member_id[ii]] * kDemand[cluster1.member_id[ii]];    //从i到ii的耗电量
                    sum += e;
                    if (e > temp_dist)
                        temp_dist = e;  //更新最大耗电量
                }
            }
        }

        //同上，计算客户在令一个簇的情况
        for (size_t ii = 0; ii < cluster2.member_id.size(); ii++) {
            if (kDemand[cluster2.member_id[ii]] < LOAD_DRONE_MAX) {
                num++;
                double e = E_COEF_DIST * 2 * kDist_D[cluster1.member_id[i]][cluster2.member_id[ii]]
                    + E_COEF_DIST_LOAD * kDist_D[cluster1.member_id[i]][cluster2.member_id[ii]] * kDemand[cluster2.member_id[ii]];
                sum += e;
                if (e > temp_dist)
                    temp_dist = e;
            }
        }
        if (sum != 0 && (sum / num) < min_all_dist) {
            result.distance = temp_dist;
            result.center_number_id = cluster1.member_id[i];
        }
    }
}


//计算簇的最小半径
MergeClusInfo eConsumed(const Cluster& cluster1, const Cluster& cluster2) { 
    double t = M;//初始化簇距离
    MergeClusInfo result(t, -1); //定义默认值
    double min_all_dist = M;  //记录最小距离
    enumerateCluster(cluster1, cluster2, result, min_all_dist);
    enumerateCluster(cluster2, cluster1, result, min_all_dist);
    return result;
}


//构建无人机路线,返回未分配的客户集合
vector<int> routeDrone(list<Route>& solution, const Cluster& cluster) {
    vector<int> set_customer;   //返回不能构建无人机路径的客户集合
    vector<int> set_customer_drone; //可以构建无人机路径的客户集合
    bool torf = false;  //记录是否可以构建无人机路线
    for (auto& customer : cluster.member_id) {
        if (customer != cluster.center_member_id && kDemand[customer] <= LOAD_DRONE_MAX) {  //如果可以构建无人机路线
            torf = true;
            set_customer_drone.push_back(customer);
        }
        else if (customer != cluster.center_member_id && kDemand[customer] > LOAD_DRONE_MAX)
            set_customer.push_back(customer);
    }
    if (!torf)
        return cluster.member_id;   //该簇没有可构建无人机路线的客户
    list<DroneRoute> temp_route_drone;   //此移动中心下的无人机路线
    bool criteria = true;   //是否还能继续合并无人机路线
    for (auto& customer : set_customer_drone) {  //构建初始无人机路线
        if (customer != cluster.center_member_id) {
            list<int> temp = { cluster.center_member_id,customer };
            double e = E_COEF_DIST * 2 * kDist_D[customer][cluster.center_member_id]
                + E_COEF_DIST_LOAD * kDist_D[customer][cluster.center_member_id] * kDemand[customer]; //初始无人机路线的耗电量
            temp_route_drone.push_back(DroneRoute(temp, kDemand[customer], e)); //  更新无人机路线
        }
    }
    while (temp_route_drone.size() > (size_t)NUM_DRONE && criteria) {
        criteria = improvedCNW(temp_route_drone);  //合并该移动卫星下的无人机路线
    }
    if (temp_route_drone.size() > (size_t)NUM_DRONE) {
        for (auto it = temp_route_drone.begin(); it != temp_route_drone.end(); it++) {
            it->time = calculateAveCost(*it);
        }
        temp_route_drone.sort(lessDroneRoute);
        while (temp_route_drone.size() > (size_t)NUM_DRONE) {
            set_customer.insert(set_customer.end(), next(temp_route_drone.back().route.begin()), temp_route_drone.back().route.end());
            temp_route_drone.erase(prev(temp_route_drone.end()));
        }
    }
    solution.push_back(Route(cluster.center_member_id, temp_route_drone));
    return set_customer;
}


bool lessDroneRoute(DroneRoute& a, DroneRoute& b) {
    return a.time < b.time;
}


double calculateAveCost(DroneRoute& a) {
    double dist = 0;
    double emission_load = 0;
    double load = a.total_demand;
    int num = 0;
    for (auto it = next(a.route.begin()); it != a.route.end(); it++) {
        num++;
        dist += kDist_D[*it][*prev(it)];
        if (next(it) == a.route.end())
            dist += kDist_D[*it][a.route.front()];
        emission_load += load * kDist_D[*prev(it)][*it];
        load -= kDemand[*it];
    }
    return (dist * COEFF_DIST_D + COEFF_EMISSION_D * (E_COEF_DIST * dist + E_COEF_DIST_LOAD * emission_load))/num;
}


//改进的C&W算法(无人机路线)
bool improvedCNW(list<DroneRoute>& solution) { 
    vector<vector<double>> cost = vector<vector<double>>(solution.size(), vector<double>(solution.size(), 0));
    vector<vector<double>> dem = vector<vector<double>>(solution.size(), vector<double>(solution.size(), 0));
    vector<vector<double>> dis = vector<vector<double>>(solution.size(), vector<double>(solution.size(), 0));
    vector<vector<double>> e = vector<vector<double>>(solution.size(), vector<double>(solution.size(), 0));
    double max_cost = -M;;    //记录最优cost
    int it1_index = 0;
    for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {   //遍历第一条无人机路线
        int it2_index = 0;
        for (auto it2 = solution.begin(); it2 != solution.end(); it2++) {   //遍历第二条无人机路线
            if (it1 != it2) {
                double temp_cost = save(*it1, *it2, it1_index, it2_index, cost, dem, dis, e);   //  计算两条路线的节省值
                if (temp_cost > max_cost) {
                    max_cost = temp_cost;
                }
            }
            it2_index++;
        }
        it1_index++;
    }

    double saving_best = -M; //记录最优节省值
    list<DroneRoute>::iterator r1_best, r2_best;   //最好合并方案
    double saving_e = 0;
    it1_index = 0;
    for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {   //遍历第一条无人机路线
        int it2_index = 0;
        for (auto it2 = solution.begin(); it2 != solution.end(); it2++) {   //遍历第二条无人机路线
            if (it1 != it2 && dem[it1_index][it2_index] >=0) {
                double temp_saving = DELTA_C * cost[it1_index][it2_index] / max_cost + DELTA_DD * exp(-dem[it1_index][it2_index])
                    + DELTA_DD * exp(-dis[it1_index][it2_index] * 0.001);
                if (saving_best < temp_saving) {
                    saving_best = temp_saving;
                    r1_best = it1;
                    r2_best = it2;
                    saving_e = e[it1_index][it2_index];
                }
            }
            it2_index++;
        }
        it1_index++;
    }
    double temp=M
    if (abs(saving_best) < temp) {
        r2_best->route.pop_front();
        r1_best->route.splice(r1_best->route.end(), r2_best->route);    //合并无人机路径
        r1_best->electricity = r1_best->electricity + r2_best->electricity - saving_e;   //更新耗电量
        r1_best->total_demand += r2_best->total_demand; //更新总需求量
        solution.erase(r2_best);
        return true;    //可以执行合并
    }
    else
        return false;   //无法执行合并
}


//计算节省值const DroneRoute& Route1, const DroneRoute& route2, int, int, vector<vector<double>>&, vector<vector<double>>&, vector<vector<double>>&, vector<vector<double>>&
double save(const DroneRoute& route1, const DroneRoute& route2, int row, int col, vector<vector<double>>& cost, vector<vector<double>>& dem, vector<vector<double>>& dis, vector<vector<double>>& e_saving) {
    double travel = 0, e = 0, dist_1 = 0, demand = 0; //初始化节省的距离、节省的电量...
    travel = kDist_D[route1.route.back()][route1.route.front()] + kDist_D[route2.route.front()][*next(route2.route.begin())]
        - kDist_D[route1.route.back()][*next(route2.route.begin())];   //节省的旅行距离
    for (auto& i : route2.route) {  //计算路径2的总需求量
        if (i != route2.route.front())
            demand += kDemand[i];
    }
    for (auto it = next(route1.route.begin()); it != route1.route.end(); it++) { //计算路径1从初始点到最后一点的距离
        dist_1 += kDist_D[*it][*prev(it)];
    }
    e = E_COEF_DIST * travel
        + E_COEF_DIST_LOAD
        * (kDist_D[route2.route.front()][*next(route2.route.begin())] - kDist_D[route1.route.back()][*next(route2.route.begin())] - dist_1) * demand;//节省的耗电量
    e_saving[row][col] = e;
    if (route1.total_demand + route2.total_demand > LOAD_DRONE_MAX || route1.electricity + route2.electricity - e > E_MAX) {
        cost[row][col] = -M;
        dem[row][col] = -M;
        dis[row][col] = -M;
    }
    else {
        cost[row][col] = COEFF_DIST_D * travel + COEFF_EMISSION_D * e;
        dem[row][col] = abs(kDemand[route1.route.back()] - kDemand[*next(route2.route.begin())]);
        dis[row][col] = kDist_D[route1.route.back()][route2.route.front()];
    }
    return cost[row][col];
}


bool improvedCNW(list<list<Route>>& solution) {
    vector<vector<double>> cost = vector<vector<double>>(solution.size(), vector<double>(solution.size(), 0));
    vector<vector<double>> dem = vector<vector<double>>(solution.size(), vector<double>(solution.size(), 0));
    vector<vector<double>> dis = vector<vector<double>>(solution.size(), vector<double>(solution.size(), 0));
    vector<vector<double>> e = vector<vector<double>>(solution.size(), vector<double>(solution.size(), 0));
    double max_cost = -M;;    //记录最优cost
    int it1_index = 0;
    for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {   //遍历第一条卡车路线
        int it2_index = 0;
        for (auto it2 = solution.begin(); it2 != solution.end(); it2++) {   //遍历第二条卡车路线
            if (it1 != it2) {
                double temp = save(*it1, *it2, it1_index, it2_index, cost, dem, dis);   //  计算两条路线的节省值
                if (max_cost < temp) {
                    max_cost = temp;
                }
            }
            it2_index++;
        }
        it1_index++;
    }

    double saving_best = -M; //记录最优节省值
    list<list<Route>>::iterator r1_best, r2_best;   //最好合并方案
    it1_index = 0;
    for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {   //遍历第一条卡车路线
        int it2_index = 0;
        for (auto it2 = solution.begin(); it2 != solution.end(); it2++) {   //遍历第二条卡车路线
            if (it1 != it2) {
                double temp_saving = DELTA_C * cost[it1_index][it2_index] / max_cost + DELTA_DD * exp(-dem[it1_index][it2_index])
                    + DELTA_DD * exp(-dis[it1_index][it2_index]*0.001);
                if (saving_best < temp_saving) {
                    saving_best = temp_saving; //记录最优方案的节省值
                    r1_best = it1;  //记录最优方案的第一路径的指针
                    r2_best = it2;  //记录最优方案的第二路径的指针
                }
            }
            it2_index++;
        }
        it1_index++;
    }
    r1_best->splice(r1_best->end(), *r2_best);    //合并无人机路径
    solution.erase(r2_best);
    return true;    //一定可以执行合并
}


//计算节省值
double save(const list<Route>& route1, const list<Route>& route2, int row, int col, vector<vector<double>>& cost, vector<vector<double>>& dem, vector<vector<double>>& dis) {
    double travel = 0, o = 0, dist_1 = 0, demand = 0; //初始化节省的距离、节省的油耗...
    travel = kDist_T[route1.back().id][kNumCustomer] + kDist_T[route2.front().id][kNumCustomer]
        - kDist_T[route1.back().id][route2.front().id];   //节省的旅行距离
    for (auto& i : route2) {
        demand += kDemand[i.id];
        demand += k_demand[i.id];  //计算广义需求量
    }
    for (auto i = route1.begin(); i != route1.end(); i++) { //计算路径1从初始点到最后一点的距离
        if (i == route1.begin())
            dist_1 += kDist_T[kNumCustomer][i->id];
        else
            dist_1 += kDist_T[i->id][prev(i)->id];
    }
    o = O_COEFF_DIST * travel
        + O_COEFF_DIST_LOAD
        * (kDist_T[route2.front().id][kNumCustomer] - kDist_T[route1.back().id][route2.front().id] - dist_1) * demand;//节省的油耗
    cost[row][col] = COEFF_DIST_T * travel + COEFF_EMISSION_T * o;
    dem[row][col] = abs(k_demand[route1.back().id] - k_demand[route2.front().id]);
    dis[row][col] = kDist_T[route1.back().id][route2.front().id];
    return cost[row][col];    //卡车节省值的组成部分
}