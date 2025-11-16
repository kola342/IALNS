#ifndef IMPROVEDALNS_H
#define IMPROVEDALNS_H

#include <list>
#include "Route.h"
#include "HyperParameter.h"
#include "GlobalVariable.h"

using namespace std;

struct MobileInfo
{
	int num;//移动卫星周围客户的数量（包括本点）
	double demand;//广义需求量
	double cost;//改移动卫星处所有无人机路线的旅行成本+耗电量成本+等待成本
	list<int> cust;//所有周围客户点的集合(包含本点)
	int mobile;//所处的移动卫星
	MobileInfo(int a, double b, list<int> cust) :num(a), demand(b), cust(cust) {};
	MobileInfo(int a, double b) :num(a), demand(b) {};
	MobileInfo(int a, double b, list<int> cust, double c, int d) :num(a), demand(b), cust(cust), cost(c), mobile(d) {};
	MobileInfo(int d) {
		num = 0;
		demand = 0;
		cust.clear();
		cost = 0;
		mobile = d;
	}
};

struct Colum
{
	int colum;
	double value;
	Colum(int colum, double value) :colum(colum), value(value) {};
};

struct IterCust
{
	int id;//客户
	char symble;//插入类型
	double time;//无人机路线时间的增加
	double e;//耗电量的增加
	double delta_cost;//成本增加
	list<DroneRoute>::iterator iter_d;
	list<int>::iterator iter_d_cust;
	list<Route>::iterator iter_t;
	IterCust(int id, double delta_cost, list<DroneRoute>::iterator iter_d, list<int>::iterator iter_d_cust, char c, list<Route>::iterator iter_t)
		:id(id), delta_cost(delta_cost), iter_d(iter_d), iter_d_cust(iter_d_cust), symble(c), iter_t(iter_t) {};
	IterCust(int id, double delta_cost, list<Route>::iterator iter_t, char c) :id(id), delta_cost(delta_cost), iter_t(iter_t), symble(c) {};
	IterCust(int id, double delta_cost, list<DroneRoute>::iterator iter_d, char c, list<Route>::iterator iter_t)
		:id(id), delta_cost(delta_cost), iter_d(iter_d), symble(c), iter_t(iter_t) {};
	IterCust(double c) :delta_cost(c) {};
};


class ImprovedALNS
{
public:
	ImprovedALNS(list<Route> solu) {
		//初始化数据
		mobile = vector<MobileInfo>(kNumCustomer, MobileInfo(0, 0));
		forward_dist = vector<double>(kNumCustomer, 0);
		backward_demand = vector<double>(kNumCustomer, 0);
		solution = solu;
		updateSolution();
		updateForwardBackward();
		calculateCost();
	}
	~ImprovedALNS() {}
	void removal_operation(const int& q, const int& symble);//移除操作
	void noise_operation(const int& symble);//噪声操作
	void insertion_operation(const int& q, const int& symble, int&);//插入操作
	double getCost() { return cost; }
	void updateEdge(double p);//更新边的历史信息

	//用于cplex
	list<Route>& getSolution() { return solution; }
	vector<MobileInfo>& getMobile() { return mobile; }
	vector<double>& getBackward() { return backward_demand; }
	void update(list<Route>::iterator, double);
	void update_less(list<Route>::iterator, double);
	void outputSolution();
	void outputSolution(vector<vector<vector<vector<bool>>>>&, vector<vector<bool>>&, vector<bool>&, vector<double>&, vector<vector<vector<vector<double>>>>&, vector<vector<double>>&);

	void intensifyOne();
	int num_cust_truk();
	void output();
	void output_truck();
	void output_weight(char);
	
private:
	list<Route> solution;//解的结构
	list<int> delete_customer;//待删除点的集合
	vector<MobileInfo> mobile;//移动卫星的相关信息
	vector<double> forward_dist;//前向距离（无人机路线的包含卡车路线，卡车路线不包含无人机路线）
	vector<double> backward_demand;//后向需求量（包含当前点）分卡车路线(广义需求量)和无人机路线
	double noise;//噪声比例
	double cost;//总成本
	double cost_travel_T;//旅行成本
	double cost_emission_T;//排放量成本
	double cost_travel_D;
	double cost_emission_D;
	double time_wait;//等待时间

	//计算必要数据

	//更新mobile数据(num,demand,cust，cost)在求解cost时需要,服务于cost,前面要updateSolution();
	void updateMobile();

	//更新前向距离以及后向需求，前面要updateSolution();
	void updateForwardBackward();

	//更新解的信息//计算无人机路线的信息，移动卫星的信息
	void updateSolution();

	//计算解的成本包括mobile的cost,内嵌updateMobile(),前面要updateSolution();
	void calculateCost();

	//检查delete_customer是否为空
	void checkDelte();

	//检查delete_customer的大小是否大于q
	void checkDelteSize(const int&);

	//执行移除操作用于非worst移除
	void Remove(const int&,list<int>, bool);

	//寻找一次相似客户点并存入delete_customer,用于shaw移除
	void shawOnce(vector<bool>&, const vector<vector<Colum>>&, bool&, bool&, int, vector<MobileInfo>&, list<int>&);

	//移除算子
	void randomRemoval(const int&);
	void shawRemoval(const int&);
	void worstRemoval(const int&);
	void worstRemovalMobile(const int&);
	void edgeInfo(const int&);
	void droneBalance(const int&);

	//噪声算子
	void zeroNoise();
	void nonZeroNoise();

	//计算各个插入位置的代价用于regreKInsertion(）和greedyInsertion()
	void calculateInsertion(list<int>::iterator, vector<IterCust>&);

	//执行插入操作
	void insert(IterCust&);
	
	//插入算子
	int regreKInsertion();
	void greedyInsertion();
	void randomGreedyInsertion();

	int selectK();

};

#endif
