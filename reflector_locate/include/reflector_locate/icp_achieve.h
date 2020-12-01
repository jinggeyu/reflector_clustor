#ifndef ICP_ACHIEVE_H_
#define ICP_ACHIEVE_H_

#include <cmath>
#include <stdio.h>
#include <eigen3/Eigen/Jacobi>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

//template <int size> 本来想用类模板 将对应点个数作为参数，但是模板参数为const，那这个太鸡肋了
class icp_achieve
{
public:
	icp_achieve(int num);
	bool compute_2D_centre(int flag, Eigen::Vector2d &data_centre);
	bool compute_2D_mean(int flag, const Eigen::Vector2d &data_centre);
	bool compute_rigid_transform();
	/* 这里没有用private，因为在register_locate中定义该类，并且需要定位计算需要用到 R T  */
	/* 如是用private，可以增加get_variable的methods */
	int correspond_size;	//对应点个数
	Eigen::Matrix2d R;
	Eigen::Vector2d T;
	Eigen::MatrixXd last_;
	Eigen::MatrixXd current_;
	Eigen::MatrixXd last_mean;	//标准化
	Eigen::MatrixXd current_mean;
};

#endif