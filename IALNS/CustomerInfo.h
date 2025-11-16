#ifndef CUSTOMERINFO_H
#define CUSTOMERINFO_H

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "HyperParameter.h"

using namespace std;

struct Customer
{
	int id;	//客户的编号
	double x;	//客户的x坐标
	double y;	//客户的y坐标

	Customer(int id = 0, double x = 0, double y = 0, double demand = 0)	//赋值函数
		:id(id), x(x), y(y) {}
};

class CustomerInfo
{
public:
	CustomerInfo(const string& filename_xy, const string& filename_demand) {	//外部读取客户信息函数
		loadCustomer(filename_xy, filename_demand);	//内部读取客户信息函数
	}
	~CustomerInfo() {}

	const vector<vector<double>>& matrixDist_D() {	//输出距离矩阵
		calculateDist_D();
		return dist_D;
	}

	const vector<vector<double>>& matrixDist_T() {	//输出距离矩阵
		calculateDist_T();
		return dist_T;
	}

	const vector<double>& getDemand() {
		return demand;
	}


private:
	vector<vector<double>> dist_D;	   //欧式距离矩阵
	vector<vector<double>> dist_T;	   //曼哈顿距离矩阵
	vector<Customer> customer;	//总客户(最后一个是原点)
	vector<double> demand;	//需求数组
	void loadCustomer(const string& filename_xy, const string& filename_demand);	//内部读取客户信息函数
	void calculateDist_D();
	void calculateDist_T();
};


#endif
