#include "CustomerInfo.h"
#include <sstream>

//读取数据
void CustomerInfo::loadCustomer(const string& filename_xy, const string& filename_demand) {
	ifstream file_xy(filename_xy);  //打开文件，构造 ifstream 对象
	ifstream file_demand(filename_demand);
	customer.clear();	//清理旧数据

	if (!file_xy) {  // 检查文件是否成功打开
		cerr << "Error opening file_xy!" << endl;
	}
	if (!file_demand) {  // 检查文件是否成功打开
		cerr << "Error opening file_demand!" << endl;
	}	

	string line;
	int id;
	double d;
	getline(file_demand, line);//忽略第一行
	while (getline(file_demand, line)) {//按空字符分割
		istringstream stream(line);
		stream >> id >> d;
		demand.push_back(d);
	}

	double x = 0, y = 0;
	getline(file_xy, line);//忽略第一行
	while (getline(file_xy, line)) {  // 使用 getline 逐行读取文件
		istringstream stream(line);
		stream >> id >> x >> y;
		customer.push_back(Customer(id, x, y));
	}
	file_demand.close();	//关闭文件
	file_xy.close();  // 关闭文件
}

void CustomerInfo::calculateDist_D() {
	dist_D.clear();	//	清理旧数据
	dist_D = vector<vector<double>>(customer.size(), vector<double>(customer.size(), 0));
	for (size_t i = 0; i < customer.size(); i++) {
		double x1 = customer.at(i).x;
		double y1 = customer.at(i).y;
		dist_D[i][i] = 0;
		for (size_t ii = i+1; ii < customer.size(); ii++) {
			double x2 = customer.at(ii).x;
			double y2 = customer.at(ii).y;
			dist_D[i][ii] = round(sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)))*1.0;
			dist_D[ii][i] = dist_D[i][ii];
		}
	}
}

void CustomerInfo::calculateDist_T() {
	dist_T.clear();	//	清理旧数据
	dist_T = vector<vector<double>>(customer.size(), vector<double>(customer.size(), 0));
	for (size_t i = 0; i < customer.size(); i++) {
		double x1 = customer.at(i).x;
		double y1 = customer.at(i).y;
		dist_T[i][i] = 0;
		for (size_t ii = i + 1; ii < customer.size(); ii++) {
			double x2 = customer.at(ii).x;
			double y2 = customer.at(ii).y;
			dist_T[i][ii] = round(abs(x1 - x2) + abs(y1 - y2))*1.0;
			dist_T[ii][i] = dist_T[i][ii];
		}
	}
}