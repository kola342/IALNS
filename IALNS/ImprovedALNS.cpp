#include "ImprovedALNS.h"
#include <iostream>
#include <vector>
#include <fstream>
#include <iomanip>
using namespace std;


//升序
static bool greaterColum(const Colum& a, const Colum& b) {
	return a.value < b.value;
}

//降序
static bool lessIterCust(const IterCust& a, const IterCust& b) {
	return a.delta_cost > b.delta_cost;
}

//升序
static bool greaterIterCust(const IterCust& a, const IterCust& b) {
	return a.delta_cost < b.delta_cost;
}

//更新移动卫星的信息
void ImprovedALNS::updateMobile() {
	cost_emission_D = 0;
	cost_travel_D = 0;
	mobile = vector<MobileInfo>(kNumCustomer, MobileInfo(0, 0));
	for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {
		int num = 1;//移动卫星的周围客户数
		double temp_demand = kDemand[it1->id];//广义需求量
		double temp_cost = 0;	//移动卫星的成本
		list<int> cust;//移动卫星周围客户点
		cust.push_back(it1->id);//移入第一个点
		for (auto& drone : it1->routes) {
			num += drone.route.size() - 1;	
			temp_demand += drone.total_demand;
			temp_cost += COEFF_DIST_D * drone.time * SPEED_D
				+ COEFF_EMISSION_D * drone.electricity;
			cost_emission_D += COEFF_EMISSION_D * drone.electricity;
			cost_travel_D += COEFF_DIST_D * drone.time * SPEED_D;
			for (auto& cust_d : drone.route) {
				if (cust_d != it1->id) {
					cust.push_back(cust_d);//计入改点到移动卫星的客户集合
					mobile[cust_d].mobile = it1->id;
				}
			}
		}
		temp_cost += COEFF_WAIT * it1->time_wait;
		mobile[it1->id] = MobileInfo(num, temp_demand, cust, temp_cost, it1->id);//记录mobile变量
	}
}

//更新前向距离和后向需求
void ImprovedALNS::updateForwardBackward() {
	int forward_cust = -3;//卡车路线的前客户点
	for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {
		//计算卡车点的前向距离
		if (it1 == solution.begin()) {
			forward_dist[it1->id] = kDist_T[kNumCustomer][it1->id];
		}
		else {
			forward_dist[it1->id] = kDist_T[forward_cust][it1->id] + forward_dist[forward_cust];
		}

		for (auto& drone : it1->routes) {
			int forward_cust_d = -2;//无人机路线的前客户点
			for (auto& cust_d : drone.route) {
				if (cust_d != it1->id) {
					forward_dist[cust_d] = kDist_D[forward_cust_d][cust_d] + forward_dist[forward_cust_d];//计算无人机点的前向距离
				}
				forward_cust_d = cust_d;//更新无人机路线的前客户点
			}
		}
		forward_cust = it1->id;//更新卡车路线的前客户点
	}
	
	for (auto it1 = solution.rbegin(); it1 != solution.rend(); it1++) {
		//计算卡车的后向需求量
		if (it1 == solution.rbegin()) {
			backward_demand[it1->id] = kDemand[it1->id];
			double demand_guangyi = 0;
			for (auto& i : it1->routes) {
				demand_guangyi += i.total_demand;
			}
			backward_demand[it1->id] += demand_guangyi;
		}
		else {
			double demand_guangyi = 0;
			for (auto& i : it1->routes) {
				demand_guangyi += i.total_demand;
			}
			demand_guangyi += kDemand[it1->id];
			backward_demand[it1->id] = backward_demand[prev(it1)->id] + demand_guangyi;
		}

		//计算无人机的后向需求
		for (auto& drone : it1->routes) {
			for (auto it3 = drone.route.begin(); it3 != drone.route.end(); it3++) {
				if (*it3 == it1->id) {
					continue;
				}
				if (*prev(it3) == it1->id) {
					backward_demand[*it3] = drone.total_demand;
				}
				else {
					backward_demand[*it3] = backward_demand[*prev(it3)] - kDemand[*prev(it3)];
				}
			}
		}
	}
}

//更新解的信息
void ImprovedALNS::updateSolution() {//计算无人机路线的信息，移动卫星的信息
	for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {
		it1->time_wait = 0;	//记录等待时间
		it1->time_second = 0;	//记录第二时间
		for (auto it2 = it1->routes.begin(); it2 != it1->routes.end(); it2++) {
			//初始化无人机路线信息
			double dis = 0;
			double total_demand = 0;
			double electricity = 0;

			//计算需求量和距离
			for (auto it3 = it2->route.begin(); it3 != it2->route.end(); it3++) {
				if (next(it3) == it2->route.end()) {
					dis += kDist_D[*it3][it1->id];
					total_demand += kDemand[*it3];
				}
				else if (it3 == it2->route.begin()) {
					dis += kDist_D[*it3][*next(it3)];
				}
				else {
					dis += kDist_D[*it3][*next(it3)];
					total_demand += kDemand[*it3];
				}
			}
			it2->time = dis / SPEED_D;
			it2->total_demand = total_demand;

			//计算耗电量
			electricity += E_COEF_DIST * dis;
			for (auto it3 = it2->route.begin(); it3 != it2->route.end(); it3++) {
				if (next(it3) == it2->route.end()) {
					continue;
				}
				electricity += E_COEF_DIST_LOAD * kDist_D[*it3][*next(it3)] * total_demand;
				total_demand -= kDemand[*next(it3)];
			}
			it2->electricity = electricity;

			//更新等待时间
			if (it2->time > it1->time_second) {
				if (it2->time > it1->time_wait) {
					it1->time_second = it1->time_wait;//更新第二等待时间
					it1->time_wait = it2->time;//更新第一等待时间
				}
				else {
					it1->time_second = it2->time;//更新第二等待时间
				}
			}
		}
	}
}

//检查移除客户集合是否为空
void ImprovedALNS::checkDelte() {
	if (!delete_customer.empty()) {
		throw exception("Removal Operator has been done!");
	}
}

//检查移除客户集合的个数是否满足q
void ImprovedALNS::checkDelteSize(const int& q) {
	if (delete_customer.size() != (unsigned)q) {
		throw exception("The number of customers removed is less than q!");
	}
}

//执行客户移除
void ImprovedALNS::Remove(const int& q, list<int> delete_c, bool check = true) {//不能改变delete_customer
	if (check) {
		checkDelteSize(q);
	}
	for (auto it1 = solution.begin(); it1 != solution.end();) {
		//判断是否移除卡车路线上的点
		auto it = find(delete_c.begin(), delete_c.end(), it1->id);
		if (it != delete_c.end()) {//判断it1是否在delete_customer中；是，则执行以下内容
			auto it_solution = it1;//记录it1
			it1++;//it1指向下一个位置
			solution.erase(it_solution);//solution中移除it1
			delete_c.erase(it);
		}
		//判断是否移除无人机路线上的点
		else {
			for (auto it2 = it1->routes.begin(); it2 != it1->routes.end(); it2++) {
				for (auto it3 = next(it2->route.begin()); it3 != it2->route.end();) {
					auto itt = find(delete_c.begin(), delete_c.end(), *it3);
					if (itt != delete_c.end()) {//判断it3是否在delete_customer中；是，则执行以下内容
						auto it = it3;//记录需要移除的点的指针
						it3++;//遍历到下一个点
						it2->route.erase(it);//移除该点
						delete_c.erase(itt);
					}
					else {
						it3++;
					}
				}
			}
			it1++;
		}
		if (delete_c.empty()) {
			return;
		}
	}
}

//执行移除操作
void ImprovedALNS::removal_operation(const int& q, const int& symbol) {
	checkDelte();
	switch (symbol)
	{
	case 0:
		randomRemoval(q);
		break;
	case 1:
		shawRemoval(q);
		break;
	case 2:
		worstRemoval(q);
		break;
	case 3:
		worstRemovalMobile(q);
		break;
	case 4:
		edgeInfo(q);
		break;
	case 5:
		droneBalance(q);
		break;
	default:
		break;
	}		
}

//每个移除算子执行完毕都需要更新前向距离和后向需求

//移除算子0
void ImprovedALNS::randomRemoval(const int& q) {//需提前知道mobile信息(num,cust)，函数结束后mobile已经改变(num),结束后更新mobile
	vector<int> position;	//记录客户
	for (size_t i = 0; i < (unsigned)kNumCustomer; i++) {
		position.push_back(i);
	}
	vector<bool> is_delete(kNumCustomer, false);//记录客户是否入选删除名单
	int num_delete = 0;
	while (num_delete != q) {
		//在客户集合中随机选择一个索引
		uniform_int_distribution<int> prob(0, position.size() - 1);
		int id = prob(gen);
		if (!is_delete[position[id]]) {//判断是否在被删除集合中；否，则执行以下操作
			if (mobile[position[id]].num == 0) {//判断是否是无人机服务的客户；是，则执行以下操作
				is_delete[position[id]] = true;//更新是否入选删除集合
				mobile[mobile[position[id]].mobile].num -= 1;//其移动卫星周围点的个数减一
				position.erase(position.begin() + id);//更新候选客户集合
				num_delete += 1;
			}
			else if (mobile[position[id]].num + num_delete <= (unsigned)q) {//检查删除移动卫星是否符合个数要求；是，则执行以下操作
				for (auto& i : mobile[position[id]].cust) {//更新是否入选删除集合
					is_delete[i] = true;
				}
				num_delete += mobile[position[id]].num;
				position.erase(position.begin() + id);//更新候选客户集合
			}
			else//本次移除操作中，不能移除的移动卫星（个数不满足要求）
				position.erase(position.begin() + id);
		}
		else//若在被删除集合中，则移出候选集合
			position.erase(position.begin() + id);
	}
	
	//将入选移除的点存入delete_customer
	for (size_t i = 0; i < is_delete.size(); i++) {
		if (is_delete[i]) {
			delete_customer.push_back(i);
		}
	}
	Remove(q, delete_customer);//执行移除操作
	updateSolution();
	updateForwardBackward();//更新前后和解
}

//移除算子1
void ImprovedALNS::shawRemoval(const int& q) {//需要提前知道mobile数据信息（num）
	vector<vector<int>> is_same_drone(kNumCustomer, vector<int>(kNumCustomer, 1));//记录是否在同一条无人机路线上
	vector<vector<int>> is_same_mobile(kNumCustomer, vector<int>(kNumCustomer, 1));//记录是否在同一移动卫星上
	//计算上述两个信息
	for (auto& customer_t : solution) {
		vector<int> mobile1;
		for (auto& drone : customer_t.routes) {
			for (auto it1 = next(drone.route.begin()); it1 != drone.route.end(); it1++) {
				mobile1.push_back(*it1);
				for (auto it2 = next(it1); it2 != drone.route.end(); it2++) {
					is_same_drone[*it1][*it2] = 0;
					is_same_drone[*it2][*it1] = 0;
				}
			}
		}
		for (auto it1 = mobile1.begin(); it1 != mobile1.end(); it1++) {
			for (auto it2 = next(it1); it2 != mobile1.end(); it2++) {
				is_same_mobile[*it1][*it2] = 0;
				is_same_mobile[*it2][*it1] = 0;
			}
		}
	}

	double max_dis_d = -M;
	double max_diff_d = -M;
	double max_dis_t = -M;
	double max_diff_t = -M;
	for (int i = 0; i < kNumCustomer; i++) {//计算上述四项
		if (mobile[i].num == 0) {//计算无人机路线的相关项
			for (int ii = i + 1; ii < kNumCustomer; ii++) {
				if (mobile[ii].num == 0) {
					if (max_dis_d < kDist_D[i][ii]) {
						max_dis_d = kDist_D[i][ii];
					}
				}
			}
		}
		else {//计算卡车路线的相关项
			for (int ii = i + 1; ii < kNumCustomer; ii++) {
				if (mobile[ii].num != 0) {
					if (max_dis_t < kDist_T[i][ii]) {
						max_dis_t = kDist_T[i][ii];
					}
				}
			}
		}
	}

	double max_t = -M;
	double min_t = M;
	double max_d = -M;
	double min_d = M;
	for (size_t i = 0; i < mobile.size(); i++) {
		if (mobile[i].num != 0) {//记录最大和最小广义需求量
			if (mobile[i].demand > max_t) {
				max_t = mobile[i].demand;
			}
			if (mobile[i].demand < min_t) {
				min_t = mobile[i].demand;
			}
		}
		else {//计算无人机路线最大最小需求量
			if (kDemand[i] > max_d) {
				max_d = kDemand[i];
			}
			if (kDemand[i] < min_d) {
				min_d = kDemand[i];
			}
		}
	}
	max_diff_d = max_d - min_d;
	max_diff_t = max_t - min_t;

	//计算相似矩阵
	vector<vector<Colum>> matrix_shaw(kNumCustomer, vector<Colum>(kNumCustomer, Colum(0, 0)));//定义相似矩阵
	for (int i = 0; i < kNumCustomer; i++) {
		matrix_shaw[i][i] = Colum(i, 0);
		if (mobile[i].num == 0) {//若无人机路线
			for (int ii = i + 1; ii < kNumCustomer; ii++) {
				if (mobile[ii].num == 0) {//若非无人机路线，则跳过本次循环
					matrix_shaw[i][ii] = Colum(ii, COEFF_SHAW_DIST * kDist_D[i][ii] / max_dis_d
						+ COEFF_SHAW_DEMAND * abs(kDemand[i] - kDemand[ii]) / max_diff_d
						+ COEFF_SHAW_DRONE * is_same_drone[i][ii]
						+ COEFF_SHAW_MOBILE * is_same_mobile[i][ii]);
					matrix_shaw[ii][i] = Colum(i, matrix_shaw[i][ii].value);
				}
			}
		}
		else {
			for (int ii = i + 1; ii < kNumCustomer; ii++) {
				if (mobile[ii].num != 0) {//若非无人机路线，则跳过本次循环
					matrix_shaw[i][ii] = Colum(ii, COEFF_SHAW_DIST * kDist_T[i][ii] / max_dis_t
						+ COEFF_SHAW_DEMAND * abs(mobile[i].demand - mobile[ii].demand) / max_diff_t);
					matrix_shaw[ii][i] = Colum(i, matrix_shaw[i][ii].value);
				}
			}
		}
	}

	vector<bool> is_delete(kNumCustomer, true);//记录是否可移除

	//随机选择一个客户
	auto temp_mobile = mobile;
	uniform_int_distribution<int> prob(0, kNumCustomer - 1);
	int first_id = prob(gen);
	while (mobile[first_id].num > q) {
		is_delete[first_id] = false;
		first_id = prob(gen);
	}
	if (mobile[first_id].num > 1) {
		for (auto& i : mobile[first_id].cust) {
			delete_customer.push_back(i);//移入第一个待删除客户
			is_delete[i] = false;//更新可删除集合
		}
	}
	else {
		delete_customer.push_back(first_id);
		is_delete[first_id] = false;
		temp_mobile[mobile[first_id].mobile].num--;
	}
	
	bool is_d = true;//是否存在无人机访问的客户
	bool is_t = true;//是否存在卡车访问的客户
	list<int> a;
	while (delete_customer.size() != (unsigned)q) {
		shawOnce(is_delete, matrix_shaw, is_d, is_t, q, temp_mobile, a);
	}

	Remove(q, delete_customer);
	updateSolution();
	updateForwardBackward();
}

//执行一次相似性选择
void ImprovedALNS::shawOnce(vector<bool>& is_delete, const vector<vector<Colum>>& matrix_shaw, bool& is_d, bool& is_t, int q, vector<MobileInfo>& temp_mobile, list<int>& a) {
	int seed_customer;//种子客户
	if (!is_d || !is_t) {
		uniform_int_distribution<int> prob(0, a.size() - 1);
		int id = prob(gen);//随机选择种子客户的index
		//定位被选中的点
		list<int>::iterator it = a.begin();
		advance(it, id);
		seed_customer = *it;
	}
	else {
		uniform_int_distribution<int> prob(0, delete_customer.size() - 1);
		int id = prob(gen);//随机选择种子客户的index
		//定位被选中的点
		list<int>::iterator it = delete_customer.begin();
		advance(it, id);
		seed_customer = *it;
	}

	vector<Colum> shaw_seed_customer;//提取与种子客户有关的所有相似性信息
	if (mobile[seed_customer].num == 0) {//种子客户被无人机访问
		for (size_t i = 0; i < matrix_shaw.size(); i++) {
			if (!is_delete[i] || i == seed_customer || mobile[i].num != 0) {//去除不可移除的客户
				continue;
			}
			shaw_seed_customer.push_back(matrix_shaw[seed_customer][i]);
		}
		if (shaw_seed_customer.empty()) {
			is_d = false;
			for (auto it = delete_customer.begin(); it != delete_customer.end(); it++) {
				if (mobile[*it].num != 0) {
					a.push_back(*it);
				}
			}
			if (a.empty()) {
				uniform_int_distribution<int> prob(0, kNumCustomer - 1);
				int first_id = prob(gen);
				while (mobile[first_id].num == 0 || !is_delete[first_id]) {
					is_delete[first_id] = false;
					first_id = prob(gen);
				}
				delete_customer.push_back(first_id);//移入第一个待删除客户
				a.push_back(first_id);
				is_delete[first_id] = false;//更新可删除集合
			}
			return;
		}
	}
	else {//种子客户被卡车访问
		for (size_t i = 0; i < matrix_shaw.size(); i++) {
			if (!is_delete[i] || i == seed_customer || mobile[i].num == 0 || (int)delete_customer.size() + temp_mobile[i].num > q) {//去除不可移除的客户
				continue;
			}
			shaw_seed_customer.push_back(matrix_shaw[seed_customer][i]);
		}
		if (shaw_seed_customer.empty()) {
			is_t = false;
			for (auto it = delete_customer.begin(); it != delete_customer.end(); it++) {
				if (mobile[*it].num == 0) {
					a.push_back(*it);
				}
			}
			if (a.empty()) {
				uniform_int_distribution<int> prob(0, kNumCustomer - 1);
				int first_id = prob(gen);
				while (mobile[first_id].num != 0 || !is_delete[first_id]) {
					is_delete[first_id] = false;
					first_id = prob(gen);
				}
				delete_customer.push_back(first_id);//移入第一个待删除客户
				a.push_back(first_id);
				is_delete[first_id] = false;//更新可删除集合
			}
			return;
		}
	}
	
	sort(shaw_seed_customer.begin(), shaw_seed_customer.end(), greaterColum);//从小到达排序
	uniform_real_distribution<double> x(0, 0.999999);
	int temp_cust = shaw_seed_customer[(int)floor(pow(x(gen), COEFF_REMOVE) * shaw_seed_customer.size())].colum;
	if (mobile[seed_customer].num == 0) {//如果为无人机服务的客户点
		delete_customer.push_back(temp_cust);//将其存入delete_customer
		is_delete[temp_cust] = false;//更新是否可移除集合
		temp_mobile[mobile[temp_cust].mobile].num--;
	}
	else if (!is_d) {
		delete_customer.push_back(temp_cust);
		a.push_back(temp_cust);
		is_delete[temp_cust] = false;
	}
	else {//如果为卡车服务的客户点
		for (auto& c : mobile[temp_cust].cust) {
			if (is_delete[c]) {
				delete_customer.push_back(c);
				is_delete[c] = false;
			}
		}
	}
}

//移除算子2
void ImprovedALNS::worstRemoval(const int& q) {//每次循环都要更新前向距离和后向载重量
	while (delete_customer.size() != (unsigned)q) {
		vector<IterCust> cost;//点的成本集合
		double dist_diff = 0, delta_cost = 0;//记录距离变化和成本变化
		double e = 0;//记录耗电量变化
		for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {
			//卡车路线的单点移除的cost计算
			if (it1->routes.empty()||it1->routes.front().route.empty()) {
				//非卡车首或卡车尾的客户点
				if (it1 != solution.begin() && next(it1) != solution.end()) {
					dist_diff = kDist_T[prev(it1)->id][it1->id] + kDist_T[it1->id][next(it1)->id] - kDist_T[prev(it1)->id][next(it1)->id];//距离的减少量
					delta_cost = COEFF_DIST_T * dist_diff//距离成本
						+ COEFF_EMISSION_T * O_COEFF_DIST * dist_diff//距离影响的排放量
						+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * dist_diff * backward_demand[next(it1)->id]//改变段的距离和载重量的乘积
						+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * forward_dist[it1->id] * kDemand[it1->id];//该点包裹产生的距离和载重量的乘积
					cost.push_back(IterCust(it1->id, delta_cost, it1, 't'));//记录移除信息
				}
				//卡车路线首部的客户点
				else if (it1 == solution.begin()) {
					dist_diff = kDist_T[kNumCustomer][it1->id] + kDist_T[it1->id][next(it1)->id] - kDist_T[kNumCustomer][next(it1)->id];//距离的变化
					delta_cost = COEFF_DIST_T * dist_diff//距离成本
						+ COEFF_EMISSION_T * O_COEFF_DIST * dist_diff//距离影响的排放量
						+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * dist_diff * backward_demand[next(it1)->id]//改变段的距离和载重量的乘积
						+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * forward_dist[it1->id] * kDemand[it1->id];//该点包裹产生的距离和载重量的乘积
					cost.push_back(IterCust(it1->id, delta_cost, it1, 't'));//记录移除信息
				}
				//在卡车路线尾部的客户点
				else {
					dist_diff = (kDist_T[prev(it1)->id][it1->id] + kDist_T[it1->id][kNumCustomer] - kDist_T[prev(it1)->id][kNumCustomer]);//距离的变化
					delta_cost = COEFF_DIST_T * dist_diff//距离成本
						+ COEFF_EMISSION_T * O_COEFF_DIST * dist_diff//距离影响的排放量
						+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * forward_dist[it1->id] * kDemand[it1->id];//该点包裹产生的距离和载重量的乘积
					cost.push_back(IterCust(it1->id, delta_cost, it1, 't'));//记录移除信息
				}
			}
			//无人机路线的移除cost计算
			else {
				for (auto it2 = it1->routes.begin(); it2 != it1->routes.end(); it2++) {
					for (auto it3 = next(it2->route.begin()); it3 != it2->route.end(); it3++) {
						//非无人机末尾的客户点情况
						if (next(it3) != it2->route.end()) {
							//移除成本计算
							dist_diff = kDist_D[*prev(it3)][*it3] + kDist_D[*it3][*next(it3)] - kDist_D[*prev(it3)][*next(it3)];//距离变化
							double cost_time = 0;//记录时间成本

							//最长无人机路线下的单点移除的时间成本计算
							if (it2->time == it1->time_wait) {
								double t = dist_diff / SPEED_D;
								if (it2->time - t < it1->time_second) {
									cost_time = (it1->time_wait - it1->time_second) * COEFF_WAIT;
								}
								else {
									cost_time = t * COEFF_WAIT;
								}
							}

							e = E_COEF_DIST * dist_diff 
								+ E_COEF_DIST_LOAD * dist_diff * backward_demand[*next(it3)] 
								+ E_COEF_DIST_LOAD * (forward_dist[*it3] - forward_dist[it1->id]) * kDemand[*it3];//耗电量的减少
							delta_cost = COEFF_DIST_D * dist_diff//无人机距离成本
								+ COEFF_EMISSION_D * e
								+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * forward_dist[it1->id] * kDemand[*it3]//该点对卡车排放的影响
								+ cost_time;//时间成本
							cost.push_back(IterCust(*it3, delta_cost, it2, it3, 'd', it1));//记录
							cost.back().e = e;
						}

						//无人机末尾的客户点情况
						else {
							double dist_diff = kDist_D[*prev(it3)][*it3] + kDist_D[*it3][it1->id] - kDist_D[*prev(it3)][it1->id];//距离变化
							double cost_time = 0;

							//最长无人机路线下的单点移除的时间成本计算
							if (it2->time == it1->time_wait) {
								double t = dist_diff / SPEED_D;
								if (it2->time - t < it1->time_second) {
									cost_time += (it1->time_wait - it1->time_second) * COEFF_WAIT;
								}
								else {
									cost_time += t * COEFF_WAIT;
								}
							}

							e = E_COEF_DIST * dist_diff + E_COEF_DIST_LOAD * (forward_dist[*it3] - forward_dist[it1->id]) * kDemand[*it3];//耗电量的减少
							double delta_cost = COEFF_DIST_D * dist_diff//无人机距离成本
								+ COEFF_EMISSION_D * e
								+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * forward_dist[it1->id] * kDemand[*it3]//该点对卡车排放的影响
								+ cost_time;//时间成本
							cost.push_back(IterCust(*it3, delta_cost, it2, it3, 'd', it1));//记录
							cost.back().e = e;
						}
					}
				}
			}
		}
		sort(cost.begin(), cost.end(), lessIterCust);//从大到小排序
		uniform_real_distribution<double> x(0, 0.999999);
		int p = (int)floor(pow(x(gen), COEFF_REMOVE) * cost.size());//带有随机性的抽取

		//执行移除操作并更新time
		//移除点为无人机服务的客户时
		if (cost[p].symble == 'd') {
			if (next(cost[p].iter_d_cust) == cost[p].iter_d->route.end()) {
				cost[p].iter_d->time -= (kDist_D[*cost[p].iter_d_cust][cost[p].iter_d->route.front()]
					+ kDist_D[*cost[p].iter_d_cust][*prev(cost[p].iter_d_cust)]
					- kDist_D[*prev(cost[p].iter_d_cust)][cost[p].iter_d->route.front()]) / SPEED_D;//无人机路线的时间减少
			}
			else {
				cost[p].iter_d->time -= (kDist_D[*cost[p].iter_d_cust][*next(cost[p].iter_d_cust)]
					+ kDist_D[*cost[p].iter_d_cust][*prev(cost[p].iter_d_cust)]
					- kDist_D[*prev(cost[p].iter_d_cust)][*next(cost[p].iter_d_cust)]) / SPEED_D;//无人机路线的时间减少
			}
			cost[p].iter_d->total_demand -= kDemand[cost[p].id];//无人机路线的需求量的减少
			cost[p].iter_d->electricity -= cost[p].e;//耗电量的减少
			delete_customer.push_back(cost[p].id);
			cost[p].iter_d->route.erase(cost[p].iter_d_cust);
			if (cost[p].iter_d->route.size() == 1) {
				cost[p].iter_t->routes.erase(cost[p].iter_d);//清空零路线
			}

			//更新第一等待时间和第二等待时间
			double t1 = 0;
			double t2 = 0;
			for (auto& drone : cost[p].iter_t->routes) {
				if (drone.time > t2) {
					if (drone.time > t1) {
						t2 = t1;//更新第二
						t1 = drone.time;//更新第一
					}
					else {
						t2 = drone.time;//更新第二
					}
				}
			}
			cost[p].iter_t->time_wait = t1;
			cost[p].iter_t->time_second = t2;
		}
		
		//移除点为卡车路线上的单点时
		else {
			if (cost[p].iter_t == solution.begin()) {
				double delta_t = kDist_T[cost[p].iter_t->id][next(cost[p].iter_t)->id]
					+ kDist_T[cost[p].iter_t->id][kNumCustomer]
					- kDist_T[kNumCustomer][next(cost[p].iter_t)->id];
			}
			else if(next(cost[p].iter_t) == solution.end()) {
				double delta_t = kDist_T[cost[p].iter_t->id][kNumCustomer]
					+ kDist_T[cost[p].iter_t->id][prev(cost[p].iter_t)->id]
					- kDist_T[prev(cost[p].iter_t)->id][kNumCustomer];
			}
			else {
				double delta_t = kDist_T[cost[p].iter_t->id][next(cost[p].iter_t)->id]
					+ kDist_T[cost[p].iter_t->id][prev(cost[p].iter_t)->id]
					- kDist_T[prev(cost[p].iter_t)->id][next(cost[p].iter_t)->id];
			}
			delete_customer.push_back(cost[p].id);
			solution.erase(cost[p].iter_t);
		}
		updateForwardBackward();
	}
}

//移除算子3
void ImprovedALNS::worstRemovalMobile(const int& q) {//需要提前知道mobile(num,cost),更新前向和后向距离
	vector<bool> not_delete = vector<bool>(kNumCustomer, false);//不能移除的点
	while (delete_customer.size() != (unsigned)q) {
		vector<IterCust> cost;
		for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {
			if (mobile[it1->id].num <= 1 || not_delete[it1->id]) {
				continue;
			}
			double dist_diff = 0;
			double temp = 0;
			if (it1 == solution.begin()) {
				dist_diff = (kDist_T[it1->id][next(it1)->id] + kDist_T[it1->id][kNumCustomer] - kDist_T[next(it1)->id][kNumCustomer]);//距离的改变
				temp = mobile[it1->id].cost//该移动卫星出无人机路线产生的总成本
					+ COEFF_DIST_T * dist_diff
					+ COEFF_EMISSION_T * O_COEFF_DIST * dist_diff
					+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * dist_diff * backward_demand[next(it1)->id]
					+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * forward_dist[it1->id] * mobile[it1->id].demand;
			}
			else if (next(it1) == solution.end()) {
				dist_diff = (kDist_T[it1->id][kNumCustomer] + kDist_T[it1->id][prev(it1)->id] - kDist_T[kNumCustomer][prev(it1)->id]);//距离的改变
				temp = mobile[it1->id].cost//该移动卫星出无人机路线产生的总成本
					+ COEFF_DIST_T * dist_diff
					+ COEFF_EMISSION_T * O_COEFF_DIST * dist_diff
					+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * forward_dist[it1->id] * mobile[it1->id].demand;
			}
			else {
				dist_diff = (kDist_T[it1->id][next(it1)->id] + kDist_T[it1->id][prev(it1)->id] - kDist_T[next(it1)->id][prev(it1)->id]);//距离的改变
				temp = mobile[it1->id].cost//该移动卫星出无人机路线产生的总成本
					+ COEFF_DIST_T * dist_diff
					+ COEFF_EMISSION_T * O_COEFF_DIST * dist_diff
					+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * dist_diff * backward_demand[next(it1)->id]
					+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * forward_dist[it1->id] * mobile[it1->id].demand;
			}		
			cost.push_back(IterCust(it1->id, temp / mobile[it1->id].num, it1, 't'));
		}
		if (cost.empty()) {
			break;
		}
		sort(cost.begin(), cost.end(), lessIterCust);//降序
		uniform_real_distribution<double> x(0, 0.999999);
		int p = (int)floor(pow(x(gen), COEFF_REMOVE) * cost.size());//随机抽取
		while ((int)delete_customer.size() + mobile[cost[p].id].num > q) {
			not_delete[cost[p].id] = true;
			cost.erase(cost.begin() + p);
			if (cost.empty()) {
				break;
			}
			p = (int)floor(pow(x(gen), COEFF_REMOVE) * cost.size());//随机抽取
		} 
		if (cost.empty()) {
			break;
		}
		for (auto& i : mobile[cost[p].id].cust) {
			delete_customer.push_back(i);
		}
		solution.erase(cost[p].iter_t);
		updateForwardBackward();
	}
	if (delete_customer.size() < (unsigned)q) {
		worstRemoval(q);
	}
}

//移除算子4
void ImprovedALNS::edgeInfo(const int& q) {//需要提前知道mobile
	list<IterCust> cost;//记录各点的值
	for (auto it = solution.begin(); it != solution.end(); it++) {
		if (it == solution.begin()) {//卡车路线第一个点的情况
			cost.push_back(IterCust(it->id, 0.5 * (edge_t[it->id][kNumCustomer] + edge_t[it->id][next(it)->id]), it, 't'));
		}
		else if (next(it) == solution.end()) {//卡车路线最后一个点的情况
			cost.push_back(IterCust(it->id, 0.5 * (edge_t[it->id][kNumCustomer] + edge_t[it->id][prev(it)->id]), it, 't'));
		}
		else {//其他情况
			cost.push_back(IterCust(it->id, 0.5 * (edge_t[it->id][next(it)->id] + edge_t[it->id][prev(it)->id]), it, 't'));
		}
		for (auto it2 = it->routes.begin(); it2 != it->routes.end(); it2++) {
			for (auto it3 = next(it2->route.begin()); it3 != it2->route.end(); it3++) {//跳过第一个点
				if (next(it3) == it2->route.end()) {//无人机路线最后一个点的情况
					cost.push_back(IterCust(*it3, 0.5 * (edge_d[*it3][*prev(it3)] + edge_d[*it3][kNumCustomer]), it2, it3, 'd', it));
				}
				else {//其他情况
					cost.push_back(IterCust(*it3, 0.5 * (edge_d[*it3][*prev(it3)] + edge_d[*it3][*next(it3)]), it2, it3, 'd', it));
				}
			}
		}
	}
	cost.sort(greaterIterCust);//升序
	vector<bool> is_delete(kNumCustomer, true);//记录是否可删除
	while (delete_customer.size() != (unsigned)q) {
		uniform_real_distribution<double> x(0, 0.999999);
		int p = (int)floor(pow(x(gen), COEFF_REMOVE) * cost.size());//随机抽取一个点
		auto it = cost.begin();
		advance(it, p);//定位选中的点
		if (is_delete[it->id]) {//判断是否可以移除
			if (mobile[it->id].num == 0) {//移除无人机点
				delete_customer.push_back(it->id);//移入待移除集合
				is_delete[it->id] = false;//更新可移除的信息
				it->iter_d->route.erase(it->iter_d_cust);//移除操作
			}
			else if ((int)delete_customer.size() + mobile[it->id].num <= q) {//移除移动卫星点
				for (auto& i : mobile[it->id].cust) {//检查周围点
					if (is_delete[i]) {//是否可移除；是，则执行以下内容
						delete_customer.push_back(i);//移入待移除集合
						is_delete[i] = false;//更新可移除的信息
					}
				}
				solution.erase(it->iter_t);
			}
		}
		cost.erase(it);//cost中移除该点
	}
	updateSolution();
	updateForwardBackward();
}

//移除算子5
void ImprovedALNS::droneBalance(const int& q) {//需要提前知道mobile数据（num，cust）
	list<IterCust> cost;//记录成本集合
	for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {
		if (mobile[it1->id].num > 1) {//是否为移动卫星；是，则执行：
			double temp = 0;
			double max = -M;
			double min = M;
			for (auto it2 = it1->routes.begin(); it2 != it1->routes.end(); it2++) {
				if (it2->time > max)
					max = it2->time;
				if (it2->time < min)
					min = it2->time;
			}
			temp = max - min;
			cost.push_back(IterCust(it1->id, temp, it1, 't'));
		}
	}
	cost.sort(lessIterCust);//降序
	while (delete_customer.size() != (unsigned)q)
	{
		if (cost.empty()) {
			break;
		}
		if (delete_customer.size() + mobile[cost.front().id].num <= q) {
			for (auto it = mobile[cost.front().id].cust.begin(); it != mobile[cost.front().id].cust.end(); it++) {
				delete_customer.push_back(*it);
			}
			solution.erase(cost.front().iter_t);
			cost.pop_front();
		}
		else {
			while (delete_customer.size() != (unsigned)q) {
				list<DroneRoute>::iterator temp_it;
				double max = 0;
				for (auto it2 = cost.front().iter_t->routes.begin(); it2 != cost.front().iter_t->routes.end(); it2++) {
					double temp = it2->time / (it2->route.size() - 1);//计算每条无人机的平均时间
					if (temp > max) {
						max = temp;
						temp_it = it2;
					}
				}
				list<int>::iterator it_delete;
				double delta_t = -1;
				for (auto it3 = next(temp_it->route.begin()); it3 != temp_it->route.end(); it3++) {
					double temp;
					if (next(it3) != temp_it->route.end()) {
						temp = kDist_D[*prev(it3)][*it3] + kDist_D[*it3][*next(it3)] - kDist_D[*prev(it3)][*next(it3)];
					}
					else {
						temp = kDist_D[*prev(it3)][*it3] + kDist_D[*it3][temp_it->route.front()] - kDist_D[*prev(it3)][temp_it->route.front()];
					}
					if (temp > delta_t) {
						delta_t = temp;
						it_delete = it3;
					}
				}
				delete_customer.push_back(*it_delete);
				temp_it->route.erase(it_delete);
				if (temp_it->route.size() == 1)
					cost.front().iter_t->routes.erase(temp_it);
				else
					temp_it->time -= delta_t / SPEED_D;
			}
		}		
	}
	if (delete_customer.size() != (unsigned)q) {
		list<IterCust> cost_t;//记录卡车成本集合
		for (auto it = solution.begin(); it != solution.end(); it++) {
			double temp;
			if (it == solution.begin()) {
				temp = kDist_T[kNumCustomer][it->id] + kDist_T[it->id][next(it)->id] - kDist_T[kNumCustomer][next(it)->id];
			}
			else if (next(it) == solution.end()) {
				temp = kDist_T[kNumCustomer][it->id] + kDist_T[it->id][prev(it)->id] - kDist_T[kNumCustomer][prev(it)->id];
			}
			else {
				temp = kDist_T[prev(it)->id][it->id] + kDist_T[it->id][next(it)->id] - kDist_T[prev(it)->id][next(it)->id];
			}
			cost_t.push_back(IterCust(it->id, temp / SPEED_T, it, 't'));
		}
		cost_t.sort(lessIterCust);//降序
		while (delete_customer.size() != (unsigned)q) {
			delete_customer.push_back(cost_t.front().id);
			list<Route>::iterator pre;
			if (cost_t.front().iter_t != solution.begin())
				pre = prev(cost_t.front().iter_t);
			else
				pre = solution.end();
			auto nex = next(cost_t.front().iter_t);
			solution.erase(cost_t.front().iter_t);
			cost_t.pop_front();
			for (auto it = cost_t.begin(); it != cost_t.end(); it++) {
				if (it->iter_t == pre || it->iter_t == nex) {
					double temp;
					auto it1 = it->iter_t;
					if (it1 == solution.begin()) {
						temp = kDist_T[kNumCustomer][it1->id] + kDist_T[it1->id][next(it1)->id] - kDist_T[kNumCustomer][next(it1)->id];
					}
					else if (next(it1) == solution.end()) {
						temp = kDist_T[kNumCustomer][it1->id] + kDist_T[it1->id][prev(it1)->id] - kDist_T[kNumCustomer][prev(it1)->id];
					}
					else {
						temp = kDist_T[prev(it1)->id][it1->id] + kDist_T[it1->id][next(it1)->id] - kDist_T[prev(it1)->id][next(it1)->id];
					}
					it->delta_cost = temp / SPEED_T;
				}
			}
			cost_t.sort(lessIterCust);//降序
		}
	}
	updateSolution();
	updateForwardBackward();
}

//插入噪声0
void ImprovedALNS::zeroNoise() {
	noise = 0;
}

//插入噪声1
void ImprovedALNS::nonZeroNoise() {
	noise = NOISE;
}

int ImprovedALNS::selectK() {
	uniform_real_distribution<double> prob(0, 1);
	double p = prob(gen);
	double sum = 0;
	for (size_t i = 0; i < weightK.size(); i++) {
		sum += weightK[i];
	}
	double t = 0;
	int k = 0;
	for (size_t i = 0; i < weightK.size(); i++) {
		t += (weightK[i] / sum);
		if (p > t)
			k = i + 1;
		else
			break;
	}
	return k + 1;
}

//插入算子0
int ImprovedALNS::regreKInsertion() {//需要实时更新前向距离和后向需求
	int k = selectK();
	while (delete_customer.size() > 0) {
		double t = -M;
		IterCust best(t);//记录最优k插入的相关信息
		list<int>::iterator it_best;//记录被插入客户
		for (auto it = delete_customer.begin(); it != delete_customer.end(); it++) {//遍历每个待插入点
			vector<IterCust> cost_iner;//插入位置的集合
			calculateInsertion(it, cost_iner);//计算后悔插入代价
			sort(cost_iner.begin(), cost_iner.end(), greaterIterCust);//升序

			//更新最大regret-k
			double cost_regretk = 0;
			if (k >= cost_iner.size()) {
				for (size_t i = 1; i < cost_iner.size(); i++) {
					cost_regretk += (cost_iner[i].delta_cost - cost_iner[0].delta_cost);
				}
			}
			else {
				for (int i = 1; i <= k; i++) {
					cost_regretk += (cost_iner[i].delta_cost - cost_iner[0].delta_cost);
				}
			}
			if (best.delta_cost < cost_regretk) {
				best = cost_iner[0];
				it_best = it;
			}
		}
		insert(best);//执行插入操作
		updateForwardBackward();//更新前向距离，后向需求
		delete_customer.erase(it_best);//更新待删除集合
	}
	return k - 1;//返回索引
}

//插入算子1
void ImprovedALNS::greedyInsertion() {
	while (delete_customer.size() > 0) {
		double t = M;
		IterCust best(t);//记录最优插入信息
		list<int>::iterator it_best;//记录最优插入的客户
		for (auto it = delete_customer.begin(); it != delete_customer.end(); it++) {
			vector<IterCust> cost_iner;//各个插入位置的信息
			calculateInsertion(it, cost_iner);
			sort(cost_iner.begin(), cost_iner.end(), greaterIterCust);

			//更新最优插入点
			if (best.delta_cost > cost_iner[0].delta_cost) {
				best = cost_iner[0];
				it_best = it;
			}
		}
		insert(best);//执行插入操作
		updateForwardBackward();//更新前向距离，后向需求
		delete_customer.erase(it_best);
	}
}

//计算各插入位置的代价
void ImprovedALNS::calculateInsertion(list<int>::iterator it, vector<IterCust>& cost_iner) {//需要提前知道(迭代更新)前向距离和后向需求量
	uniform_real_distribution<double> prob(-noise, noise);//噪声取值比例范围
	double dist_diff = 0, cost = 0;//存放距离和增加成本
	double e = 0;//存放增加的无人机耗电量
	if (solution.empty()) {
		dist_diff = 2*kDist_T[kNumCustomer][*it];//距离改变
		cost = COEFF_DIST_T * dist_diff//距离变化的成本
			+ COEFF_EMISSION_T * O_COEFF_DIST * dist_diff//油耗影响的距离变化部分
			+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * kDist_T[kNumCustomer][*it] * kDemand[*it];//新增客户对前向的距离载重乘积的影响
		cost_iner.push_back(IterCust(*it, cost * (1 + prob(gen)), solution.end(), 't'));
		return;
	}
	for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {
		//插入在卡车第一个位置的情况
		if (it1 == solution.begin()) {
			dist_diff = kDist_T[kNumCustomer][*it] + kDist_T[*it][it1->id] - kDist_T[it1->id][kNumCustomer];//距离改变
			cost = COEFF_DIST_T * dist_diff//距离变化的成本
				+ COEFF_EMISSION_T * O_COEFF_DIST * dist_diff//油耗影响的距离变化部分
				+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * dist_diff * backward_demand[it1->id]//距离变化对当前段的距离载重乘积的影响
				+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * kDist_T[kNumCustomer][*it] * kDemand[*it];//新增客户对前向的距离载重乘积的影响
			cost_iner.push_back(IterCust(*it, cost * (1 + prob(gen)), it1, 't'));
		}

		//其他情况
		else {
			dist_diff = kDist_T[prev(it1)->id][*it] + kDist_T[*it][it1->id] - kDist_T[it1->id][prev(it1)->id];//距离改变
			cost = COEFF_DIST_T * dist_diff//距离变化的成本
				+ COEFF_EMISSION_T * O_COEFF_DIST * dist_diff//油耗影响的距离变化部分
				+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * dist_diff * backward_demand[it1->id]//距离变化对当前段的距离载重乘积的影响
				+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * (forward_dist[prev(it1)->id] + kDist_T[prev(it1)->id][*it]) * kDemand[*it];//新增客户对前向的距离载重乘积的影响
			cost_iner.push_back(IterCust(*it, cost * (1 + prob(gen)), it1, 't'));
		}

		//插入在卡车末尾的情况
		if (next(it1) == solution.end()) {
			dist_diff = kDist_T[kNumCustomer][*it] + kDist_T[*it][it1->id] - kDist_T[it1->id][kNumCustomer];//距离改变
			cost = COEFF_DIST_T * dist_diff//距离变化的成本
				+ COEFF_EMISSION_T * O_COEFF_DIST * dist_diff//油耗影响的距离变化部分
				+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * (forward_dist[it1->id] + kDist_T[it1->id][*it]) * kDemand[*it];//新增客户对前向的距离载重乘积的影响
			cost_iner.push_back(IterCust(*it, cost * (1 + prob(gen)), solution.end(), 't'));
		}

		//插入在无人机路线的情况
		if (kDemand[*it] > LOAD_DRONE_MAX) {
			continue;
		}
		vector<list<DroneRoute>::iterator> delete_it2;
		for (auto it2 = it1->routes.begin(); it2 != it1->routes.end(); it2++) {
			//需要先检验空路线的情况
			if (it2->route.size() == 1) {
				delete_it2.push_back(it2);//记录该无人机路线
				continue;
			}
			if (it2->total_demand + kDemand[*it] > LOAD_DRONE_MAX) {
				continue;	//符合载重约束时才能执行
			}
			for (auto it3 = next(it2->route.begin()); it3 != it2->route.end(); it3++) {
				//插入在无人机路线的非末尾的情况
				//需要先检验插入在无人机路线的末尾的情况
				if (next(it3) == it2->route.end()) {
					dist_diff = kDist_D[it1->id][*it] + kDist_D[*it][*it3] - kDist_D[it1->id][*it3];//距离改变量
					e= E_COEF_DIST * dist_diff
						+ E_COEF_DIST_LOAD * (kDist_D[*it3][*it] + forward_dist[*it3] - forward_dist[it1->id]) * kDemand[*it];//耗电量增加
					if (it2->electricity + e <= E_MAX) {//判断是否还能插入
						cost = COEFF_DIST_D * dist_diff
							+ COEFF_EMISSION_D * e
							+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * forward_dist[it1->id] * kDemand[*it];

						//判断并计算时间成本
						if (it2->time + dist_diff / SPEED_D > it1->time_wait) {
							cost += (it2->time + dist_diff / SPEED_D - it1->time_wait) * COEFF_WAIT;
						}

						cost_iner.push_back(IterCust(*it, cost * (1 + prob(gen)), it2, it2->route.end(), 'd', it1));
						cost_iner.back().time = dist_diff / SPEED_D;//记录时间改变量
						cost_iner.back().e = e;
					}
				}

				dist_diff = kDist_D[*prev(it3)][*it] + kDist_D[*it][*it3] - kDist_D[*prev(it3)][*it3];
				e = E_COEF_DIST * dist_diff
					+ E_COEF_DIST_LOAD * dist_diff * backward_demand[*it3]
					+ E_COEF_DIST_LOAD * (forward_dist[*prev(it3)] - forward_dist[it1->id] + kDist_D[*prev(it3)][*it]) * kDemand[*it];
				if (it2->electricity + e <= E_MAX) {
					cost = COEFF_DIST_D * dist_diff
						+ COEFF_EMISSION_D * e
						+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * forward_dist[it1->id] * kDemand[*it];

					//判断并计算时间成本
					if (it2->time + dist_diff / SPEED_D > it1->time_wait) {
						cost += (it2->time + dist_diff / SPEED_D - it1->time_wait) * COEFF_WAIT;
					}

					cost_iner.push_back(IterCust(*it, cost * (1 + prob(gen)), it2, it3, 'd', it1));
					cost_iner.back().time = dist_diff / SPEED_D;//记录时间改变量
					cost_iner.back().e = e;
				}
			}
		}

		//删除空无人机路线
		for (auto& i : delete_it2) {
			it1->routes.erase(i);
		}

		//开辟新的无人机路线的情况
		if (it1->routes.size() < (unsigned)NUM_DRONE && kDemand[*it] <= LOAD_DRONE_MAX) {
			dist_diff = 2 * kDist_D[it1->id][*it];
			e = E_COEF_DIST * dist_diff + E_COEF_DIST_LOAD * kDist_D[it1->id][*it] * kDemand[*it];
			if (e <= E_MAX) {
				cost = COEFF_DIST_D * dist_diff
					+ COEFF_EMISSION_D * e
					+ COEFF_EMISSION_T * O_COEFF_DIST_LOAD * forward_dist[it1->id] * kDemand[*it];
				if (dist_diff / SPEED_D > it1->time_wait) {
					cost += (dist_diff / SPEED_D - it1->time_wait) * COEFF_WAIT;
				}
				cost_iner.push_back(IterCust(*it, cost * (1 + prob(gen)), it1->routes.end(), 'd', it1));
				cost_iner.back().time = dist_diff / SPEED_D;//记录时间改变量
				cost_iner.back().e = e;
			}
		}
	}
}

//执行插入操作
void ImprovedALNS::insert(IterCust& best) {
	if (best.symble == 't') {//如果插入卡车路线
		solution.insert(best.iter_t, Route(best.id));//插入操作
	}
	else {//如果插入无人机路线
		if (best.iter_d == best.iter_t->routes.end()) {//如果新增无人机路线
			list<int> temp = { best.iter_t->id,best.id };//构造新路线
			DroneRoute temp_dr(best.time, temp);
			temp_dr.electricity = best.e;//耗电量的更新
			temp_dr.total_demand = kDemand[best.id];//更新无人机路线的总需求
			best.iter_t->routes.insert(best.iter_d, temp_dr);//插入操作（时间和路线）
			if (best.time > best.iter_t->time_second) {
				if (best.time > best.iter_t->time_wait) {
					best.iter_t->time_second = best.iter_t->time_wait;
					best.iter_t->time_wait = best.time;//更新等待时间
				}
				else {
					best.iter_t->time_second = best.time;
				}
			}
		}
		else {//如果没有新增无人机路线
			best.iter_d->route.insert(best.iter_d_cust, best.id);//插入操作
			best.iter_d->time += best.time;//更新时间
			best.iter_d->electricity += best.e;//耗电量的更新
			best.iter_d->total_demand += kDemand[best.id];//更新无人机路线的总需求
			if (best.iter_d->time > best.iter_t->time_second) {
				if (best.iter_d->time > best.iter_t->time_wait) {
					best.iter_t->time_second = best.iter_t->time_wait;
					best.iter_t->time_wait = best.iter_d->time;//更新等待时间
				}
				else {
					best.iter_t->time_second = best.iter_d->time;
				}
			}
		}
	}
}

//插入算子2
void ImprovedALNS::randomGreedyInsertion() {
	while (delete_customer.size()>0) {
		uniform_int_distribution<int> prob(0, delete_customer.size() - 1);
		int n = prob(gen);//随机抽取delete_customer中的index
		auto it = delete_customer.begin();
		advance(it, n);//定位index
		double t = M;
		IterCust best(t);//记录插入成本最小的
		vector<IterCust> cost_iner;//各个插入位置的信息
		calculateInsertion(it, cost_iner);//计算各个插入位置的成本
		for (auto& i : cost_iner) {
			if (i.delta_cost < best.delta_cost) {
				best = i;
			}
		}//更新最优插入的信息

		insert(best);//执行插入操作
		updateForwardBackward();//更新前向距离，后向需求
		delete_customer.erase(it);
	}
}

//计算接的成本信息
void ImprovedALNS::calculateCost() {
	//初始化成本数据
	cost_emission_T = 0;
	cost_travel_T = 0;
	cost = 0;
	time_wait = 0;

	updateMobile();//计算移动卫星的总成本
	for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {
		//计算卡车的旅行成本和排放成本
		if (it1 == solution.begin()) {
			cost_travel_T += COEFF_DIST_T * kDist_T[kNumCustomer][it1->id];
			cost_emission_T += COEFF_EMISSION_T * (O_COEFF_DIST * kDist_T[kNumCustomer][it1->id]
				+ O_COEFF_DIST_LOAD * kDist_T[kNumCustomer][it1->id] * backward_demand[it1->id]);
		}
		else {
			cost_travel_T += COEFF_DIST_T * kDist_T[prev(it1)->id][it1->id];
			cost_emission_T += COEFF_EMISSION_T *( O_COEFF_DIST * kDist_T[prev(it1)->id][it1->id]
				+ O_COEFF_DIST_LOAD * kDist_T[prev(it1)->id][it1->id] * backward_demand[it1->id]);
		}
		if (next(it1)==solution.end()) {
		cost_travel_T += COEFF_DIST_T * kDist_T[kNumCustomer][it1->id];
		cost_emission_T += COEFF_EMISSION_T * (O_COEFF_DIST * kDist_T[kNumCustomer][it1->id]);
		}
		//加上移动卫星的总成本
		cost += mobile[it1->id].cost;
		time_wait += it1->time_wait;
	}
	cost += cost_emission_T + cost_travel_T;
}

//执行插入操作
void ImprovedALNS::insertion_operation(const int& q, const int& symbol, int& regret_k) {
	checkDelteSize(q);
	switch (symbol)
	{
	case 0:
		regret_k = regreKInsertion();//返回索引
		break;
	case 1:
		greedyInsertion();
		break;
	case 2:
		randomGreedyInsertion();
		break;
	default:
		break;
	}	
	calculateCost();
}

//执行噪声操作
void ImprovedALNS::noise_operation(const int& symble) {
	switch (symble)
	{
	case 0:
		zeroNoise();
		break;
	case 1:
		nonZeroNoise();
		break;
	default:
		break;
	}
}

//更新边的历史信息
void ImprovedALNS::updateEdge(double p) {
	for (auto& row : edge_t) {
		for (auto& colum : row) {
			colum = colum * R_EDGE;
		}
	}
	for (auto& row : edge_d) {
		for (auto& colum : row) {
			colum = colum * R_EDGE;
		}
	}
	for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {
		if (it1 == solution.begin()) {
			edge_t[kNumCustomer][it1->id] += p;
		}
		else {
			edge_t[prev(it1)->id][it1->id] += p;
		}
		if (next(it1) == solution.end()) {
			edge_t[it1->id][kNumCustomer] += p;
		}
		for (auto it2 = it1->routes.begin(); it2 != it1->routes.end(); it2++) {
			for (auto it3 = it2->route.begin(); it3 != it2->route.end(); it3++) {
				if (next(it3) == it2->route.end()) {
					edge_d[*it3][kNumCustomer] += p;
				}
				else {
					edge_d[*it3][*next(it3)] += p;
				}
			}
		}
	}
}

void ImprovedALNS::output() {
	for (auto& i : solution) {
		std::cout << i.id+1 << "--";
		for (auto& ii : i.routes) {
			std::cout << "无人机路线：";
			for (auto& iii : ii.route) {
				std::cout << iii+1 << "--";
			}
			std::cout << "  时间：" << ii.time;
			std::cout << "  耗电量：" << ii.electricity;
			std::cout << "  总需求" << ii.total_demand;
		}
		std::cout << endl;
	}
	for (size_t i = 0; i < mobile.size();i++) {
		if (mobile[i].num > 1) {
			std::cout << "移动卫星客户" << i << "的成本: " << mobile[i].cost;
			std::cout << endl;
		}
	}
	for (size_t i = 0; i < mobile.size(); i++) {
		std::cout << "客户" << i << "的广义需求: " << mobile[i].demand;
		std::cout << endl;
	}
	for (size_t i = 0; i < forward_dist.size(); i++) {
		std::cout << "客户" << i << "的前向距离: " << forward_dist[i];
		std::cout << endl;
	}
	for (size_t i = 0; i < backward_demand.size(); i++) {
		std::cout << "客户" << i << "的后向需求: " << backward_demand[i];
		std::cout << endl;
	}
	std::cout << "总成本：" << cost << endl;
	std::cout << "卡车旅行成本：" << cost_travel_T << endl;
	std::cout << "卡车排放成本：" << cost_emission_T << endl;
	std::cout << "卡车等待成本：" << time_wait*COEFF_WAIT << endl;
	std::cout << "无人机旅行成本：" << cost_travel_D << endl;
	std::cout << "无人机排放成本：" << cost_emission_D << endl;
}

int ImprovedALNS::num_cust_truk() {
	return solution.size();
}

void ImprovedALNS::output_truck() {
	for (auto& i : solution) {
		std::cout << i.id;
		if (mobile[i.id].num > 1) {
			std::cout << "**";
		}
		std::cout << "--";
	}
	std::cout << endl;
}

void ImprovedALNS::output_weight(char s) {
	switch (s) {
	case 'r':
		for (size_t i = 0; i < kWeightRemoval.size(); i++) {
			std::cout << kWeightRemoval[i] << "--";
		}
		cout << endl;
		break;
	case 'n':
		for (size_t i = 0; i < kWeightNoise.size(); i++) {
			std::cout << kWeightNoise[i] << "--";
		}
		cout << endl;
		break;
	case 'i':
		for (size_t i = 0; i < kWeightInsertion.size(); i++) {
			std::cout << kWeightInsertion[i] << "--";
		}
		cout << endl;
		break;
	case 'k':
		for (size_t i = 0; i < weightK.size(); i++) {
			std::cout << weightK[i] << "--";
		}
		cout << endl;
		break;
	default:
		break;
	}
	
}

void ImprovedALNS::update(list<Route>::iterator it, double a) {//需要更改time_second,每条路径上的电量，时间，前向后向。
	it->time_second = 0;
	for (auto it1 = it->routes.begin(); it1 != it->routes.end(); it1++) {
		it1->electricity = 0;
		for (auto it2 = next(it1->route.begin()); it2 != it1->route.end(); it2++) {
			forward_dist[*it2] = forward_dist[*prev(it2)] + kDist_D[*prev(it2)][*it2];//前向
			if (*prev(it2) == it->id) {
				backward_demand[*it2] = it1->total_demand;
			}
			else {
				backward_demand[*it2] = backward_demand[*prev(it2)] - kDemand[*prev(it2)];//后向
			}
			it1->electricity += E_COEF_DIST_LOAD * backward_demand[*it2] * kDist_D[*prev(it2)][*it2];
		}
		double travel = forward_dist[*prev(it1->route.end())] - forward_dist[it->id] + kDist_D[it->id][*prev(it1->route.end())];
		it1->electricity += travel * E_COEF_DIST;
		it1->time = travel / SPEED_D;//每条路径上的时间
		if (it1->time<it->time_wait && it1->time>it->time_second)
			it->time_second = it1->time;//更改time_second
	}
	cost = cost - a + mobile[it->id].cost;//更新解
}

void ImprovedALNS::update_less(list<Route>::iterator it, double a) {//只更新解和前后向
	for (auto it1 = it->routes.begin(); it1 != it->routes.end(); it1++) {
		for (auto it2 = next(it1->route.begin()); it2 != it1->route.end(); it2++) {
			forward_dist[*it2] = forward_dist[*prev(it2)] + kDist_D[*prev(it2)][*it2];//前向
			if (*prev(it2) == it->id) {
				backward_demand[*it2] = it1->total_demand;
			}
			else {
				backward_demand[*it2] = backward_demand[*prev(it2)] - kDemand[*prev(it2)];//后向
			}
		}
	}
	cost = cost - a + mobile[it->id].cost;//更新解
}

void ImprovedALNS::intensifyOne() {
	//移除卡车路径上的非移动卫星
	for (auto it = solution.begin(); it != solution.end();) {
		if (mobile[it->id].num == 1) {
			delete_customer.push_back(it->id);
			it = this->solution.erase(it);
		}
		else {
			it++;
		}
	}
	//选择插入算子
	std::uniform_real_distribution<double> prob3(0, 1);
	double p = prob3(gen);
	double sum = 0;
	for (size_t i = 0; i < kWeightInsertion.size(); i++) {
		sum += kWeightInsertion[i];
	}
	double t = 0;
	int symbol = 0;
	for (int i = 0; i < 3; i++) {
		t += kWeightInsertion[i] / sum;
		if (p > t)
			symbol = i + 1;
		else
			break;
	}
	int regret_k = -1;
	this->insertion_operation((int)delete_customer.size(), symbol, regret_k);
}

void ImprovedALNS::outputSolution() {

	// 打开文件以写入数据
	ofstream outFile(SOLUTION);//创建并打开文件，如果文件不存在会自动创建

	if (!outFile) {
		std::cerr << "无法打开文件!" << std::endl;
		return;
	}

	//outFile << std::fixed << std::setprecision(4) << cost << "\n";
	for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {
		if (it1 == solution.begin()) {
			outFile << "x[" << 0 << "," << it1->id + 1 << "]=" << 1 << "\n";
			outFile << "qT[" << 0 << "," << it1->id + 1 << "]=" << std::fixed << std::setprecision(14) << backward_demand[it1->id] << "\n";
		}
		if (next(it1) == solution.end()) {
			outFile << "x[" << it1->id + 1 << "," << 0 << "]=" << 1 << "\n";
		}
		else {
			outFile << "x[" << it1->id + 1 << "," << next(it1)->id + 1 << "]=" << 1 << "\n";
			outFile << "qT[" << it1->id + 1 << "," << next(it1)->id + 1 << "]=" << std::fixed << std::setprecision(14) << backward_demand[next(it1)->id] << "\n";
		}
	}

	for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {
		if (!it1->routes.empty()) {
			outFile << "t[" << it1->id + 1 << "]=" << std::fixed << std::setprecision(14) << it1->time_wait << "\n";
		}
	}

	for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {
		outFile << "w[" << it1->id + 1 << "]=" << 1 << "\n";
	}

	for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {
		int d = 0;
		for (auto it2 = it1->routes.begin(); it2 != it1->routes.end(); it2++) {
			for (auto it3 = it2->route.begin(); it3 != it2->route.end(); it3++) {
				if (next(it3) == it2->route.end()) {
					outFile << "y[" << (*it3) + 1 << "," << it1->id + 1 << "," << d << "," << it1->id + 1 << "]=" << 1 << "\n";
				}
				else {
					outFile << "y[" << (*it3) + 1 << "," << *next(it3) + 1 << "," << d << "," << it1->id + 1 << "]=" << 1 << "\n";
					outFile << "qD[" << (*it3) + 1 << "," << *next(it3) + 1 << "," << d << "," << it1->id + 1 << "]=" << std::fixed << std::setprecision(14) << backward_demand[*next(it3)] << "\n";
				}
			}
			d++;
		}
	}

	// 关闭文件
	outFile.close();

	std::cout << "Data has been written to 'data_output.txt'." << endl;
}

void ImprovedALNS::outputSolution(vector<vector<vector<vector<bool>>>>& y, vector<vector<bool>>& x, vector<bool>& w, vector<double>& t, vector<vector<vector<vector<double>>>>& qD, vector<vector<double>>& qT) {
	y = vector<vector<vector<vector<bool>>>>(kNumCustomer + 1, vector<vector<vector<bool>>>(kNumCustomer + 1, vector<vector<bool>>(NUM_DRONE, vector<bool>(kNumCustomer + 1, 0))));
	x = vector<vector<bool>>(kNumCustomer + 1, vector<bool>(kNumCustomer + 1, 0));
	w = vector<bool>(kNumCustomer + 1, 0);
	t = vector<double>(kNumCustomer + 1, 0);
	qD = vector<vector<vector<vector<double>>>>(kNumCustomer + 1, vector<vector<vector<double>>>(kNumCustomer + 1, vector<vector<double>>(NUM_DRONE, vector<double>(kNumCustomer + 1, 0))));
	qT = vector<vector<double>>(kNumCustomer + 1, vector<double>(kNumCustomer + 1, 0));

	for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {
		if (it1 == solution.begin()) {
			x[0][it1->id + 1] = 1;
			qT[0][it1->id + 1] = backward_demand[it1->id];
		}
		if (next(it1) == solution.end()) {
			x[it1->id + 1][0] = 1;
		}
		else {
			x[it1->id + 1][next(it1)->id + 1] = 1;
			qT[it1->id + 1][next(it1)->id + 1] = backward_demand[next(it1)->id];
		}
	}

	for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {
		if (!it1->routes.empty()) {
			t[it1->id + 1] = it1->time_wait;
		}
	}

	for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {
		w[it1->id + 1] = 1;
	}

	for (auto it1 = solution.begin(); it1 != solution.end(); it1++) {
		int d = 0;
		for (auto it2 = it1->routes.begin(); it2 != it1->routes.end(); it2++) {
			for (auto it3 = it2->route.begin(); it3 != it2->route.end(); it3++) {
				if (next(it3) == it2->route.end()) {
					y[(*it3) + 1][it1->id + 1][d][it1->id + 1] = 1;
				}
				else {
					y[(*it3) + 1][*next(it3) + 1][d][it1->id + 1] = 1;
					qD[(*it3) + 1][*next(it3) + 1][d][it1->id + 1] = backward_demand[*next(it3)];
					}
			}
			d++;
		}
	}
}