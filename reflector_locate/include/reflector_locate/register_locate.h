#ifndef REGISTER_REFLECTOR_H_
#define REGISTER_REFLECTOR_H_

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>

#include <cmath>
#include <stdio.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include "reflector_locate/reflector_msgs.h"
#include "reflector_locate/icp_achieve.h"

/* 距离矩阵匹配阈值 */
#define Distance_Error 0.25 //9.5cm
/* 位姿预估匹配 距离、角度阈值 */
#define Pre_dis_error 0.25
#define Pre_ang_error 0.18	//10度

#define Pi 3.1415926
#define predict_period 0.2 //200ms

/*  构建反光板局部了列表的flag */
#define set_last 0	//ref_last
#define set_current 1	//ref_current

using namespace std;
//using namespace Eigen;

//反光板的坐标
//float ref_reflector[5][2] = {
//	{-0.297, 1.598}, { 1.315, 1.600}, { 1.323, -0.410}, { -1.170, -1.725}, {-1.415, 0.000}
//};

float ref_reflector[5][2] = {
	{ 1.180, 0.200}, { 1.600, - 1.315}, { 0.000, -1.323}, { -1.240, -0.570}, {-0.378, 1.415}
};

//float ref_reflector[5][2] = {
//	{-0.7, 1.2}, { 1.315, 1.600}, { 1.323, 0.000}, { 0.523, -1.325}, {-1.415, -0.400}
//};

/* 用来测试一次位姿计算耗时 在动态register_ID中调用
ros::Time time_last;
ros::Time time_current;
double predict_period_;
*/

class RegisterReflector
{
public:
	RegisterReflector();
	~RegisterReflector();
	bool init();
	//重载距离计算函数（参数类型、参数个数）
	float calc_dist(float x1, float y1, float x2, float y2);
	float calc_dist(float d1, float d2, float th);
	
	//重载反光板匹配
	bool regist_ID(int det_num); // 距离矩阵方法：检测反光板与参考反光板进行匹配
	bool regist_ID(int det_num, const reflector_locate::reflector_msgs &msg);	// 动态匹配方法

	void pose_predict_func();	//预估机器人位姿
	void set_ICP_list(int num, const reflector_locate::reflector_msgs &msg, int flag);	//将（d,theta）转换为 局部(x, y)

	/* linearSVD ：用于静态初始定位 以及 定位失败后的重新定位
		ICP				  ：动态定位		*/
	bool compute_pose_linearSVD(const reflector_locate::reflector_msgs &msg);	//使用线性SVD方法进行求解
	bool compute_pose_ICP();	//使用ICP的原理进行求解

	void MoveInfoCallback(const geometry_msgs::Twist &msg); 	
	void DetReflectorCallback(const reflector_locate::reflector_msgs &msg);

	void pub_path(const Eigen::Vector3d &pose, nav_msgs::Path &path, ros::Publisher &_path_pub);
private:
	ros:: NodeHandle nh_;
	ros::Subscriber det_reflector_sub_; //订阅反光板检测信息
	ros::Subscriber move_info_sub_; //订阅agv的移动信息 /cmd_vel 用于预估位姿

	//发布两种计算方法下的运动轨迹
	ros::Publisher SVD_path_pub;	
	ros::Publisher ICP_path_pub;

	nav_msgs::Path SVD_path;
	nav_msgs::Path ICP_path;

//输出x y 位置，matlab处理.bag文件
	ros::Publisher SVD_pose_pub;
	ros::Publisher ICP_pose_pub;

	geometry_msgs::Point SVD_pose;
	geometry_msgs::Point ICP_pose;

	static bool Initial_Flag;	//false：静态定位 true：动态定位过程
	
/* 下面两种匹配均没有对误检反光板做剔除处理，认为有错误就直接扔掉整个数据，匹配下一帧 */
//反光板静态匹配
	Eigen::MatrixXf ref_dist;	//参考距离矩阵
	Eigen::MatrixXf det_dist;	//检测距离矩阵

	Eigen::VectorXi Table_ID_last;	//前一时刻匹配ID表
	Eigen::VectorXi Table_ID;	//当前反光板匹配ID表

//反光板动态匹配
	Eigen::Vector3d pose_last; //上一时刻位姿
	Eigen::Vector3d pose_predict; //预估位姿
	geometry_msgs::Twist v_w_; // 机器人的速度信息 float64

//构建ICP计算的反光板局部坐标矩阵
	Eigen::MatrixXd ref_last; //记录上一时刻的反光板局部坐标矩阵
	Eigen::MatrixXd ref_current;
};

#endif
