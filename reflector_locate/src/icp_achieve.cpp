#include "reflector_locate/icp_achieve.h"

//template <int size>
icp_achieve::icp_achieve(int num):correspond_size(num)
{
	R.setZero();
	T.setZero();
	last_.resize(2, correspond_size);
	current_.resize(2, correspond_size);
	last_mean.resize(2, correspond_size);
	current_mean.resize(2, correspond_size);
	last_.setZero();
	current_.setZero();
	last_mean.setZero();
	current_mean.setZero();
}

/* 计算中心 */
//template <int size>
bool icp_achieve::compute_2D_centre(int flag, Eigen::Vector2d &data_centre)
{
	Eigen::MatrixXd data_(2, correspond_size);
	if(flag == 0) 
		data_ = last_;
	else 
		data_ = current_;
	double x_ = 0.0;
	double y_ = 0.0;
	for(int i=0; i<correspond_size; i++)
	{
		x_ += data_(0, i);
		y_ += data_(1, i);
	}
	data_centre(0) = x_/correspond_size;
	data_centre(1) = y_/correspond_size;
}

/* 标准化 */
//template <int size>
bool icp_achieve::compute_2D_mean(int flag, const Eigen::Vector2d &data_centre)
{
	Eigen::MatrixXd data_(2, correspond_size);
	Eigen::MatrixXd data_mean(2, correspond_size);
	if(flag == 0) 
		data_ = last_;
	else 
		data_ = current_;
	for(int i=0; i<correspond_size; i++)
	{
		data_mean(0, i) = data_(0, i) - data_centre(0);
		data_mean(1, i) = data_(1, i) - data_centre(1);
	}
	if(flag == 0)
		last_mean = data_mean;
	else
		current_mean = data_mean;
}

/* 计算刚体变换 */
//template <int size>
bool icp_achieve::compute_rigid_transform()
{
	Eigen::Vector2d last_centre;
	Eigen::Vector2d current_centre;
	compute_2D_centre(0, last_centre);	//last_
	compute_2D_centre(1, current_centre);	//current_

	compute_2D_mean(0, last_centre);
	compute_2D_mean(1, current_centre);

	Eigen::Matrix<double, 2, 2> H = last_mean*current_mean.transpose();
	Eigen::JacobiSVD<Eigen::Matrix<double, 2, 2> > svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix<double, 2, 2> u = svd.matrixU ();
  	Eigen::Matrix<double, 2, 2> v = svd.matrixV ();

  	R = v * u.transpose ();
  	if(R.determinant() < 0)		//特征值小于0，why？
  	{
  		for(int i=0; i<2; i++)
  		{
  			v(i, 1) *= -1;
  		}
  		R = v * u.transpose ();
  	}
  	T = last_centre - R*current_centre;
}