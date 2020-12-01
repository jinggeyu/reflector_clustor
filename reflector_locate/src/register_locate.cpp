#include "reflector_locate/register_locate.h"	//要说明.h的路径 reflector_locate/
#include  <linux/string.h>

bool RegisterReflector::Initial_Flag = false; //反光板匹配初始状态标识

RegisterReflector::RegisterReflector()
{
    //Init gazebo ros turtlebot3 node
	ROS_INFO("RegisterReflector Node Init");
	ROS_ASSERT(init());
}

RegisterReflector::~RegisterReflector()
{
	ros::shutdown();
}

float RegisterReflector::calc_dist(float x1, float y1, float x2, float y2)
{
	return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

float RegisterReflector::calc_dist(float d1, float d2, float th)
{
	return sqrt(d1*d1 + d2*d2 - 2*d1*d2*cos(th));
}

bool RegisterReflector::init()
{
//	Initial_Flag = false;	//反光板匹配初始状态标识
	ref_dist.resize(5,4);
	det_reflector_sub_  = nh_.subscribe("reflector_pose", 10, &RegisterReflector::DetReflectorCallback, this);
	move_info_sub_ = nh_.subscribe("/cmd_vel", 10, &RegisterReflector::MoveInfoCallback, this);

	SVD_path_pub = nh_.advertise<nav_msgs::Path>("SVD_path", 1, true);
	ICP_path_pub = nh_.advertise<nav_msgs::Path>("ICP_path", 1, true);

	SVD_path.header.stamp = ros::Time::now();
	SVD_path.header.frame_id = "odom";	//pointcloud坐标系
	ICP_path.header.stamp = ros::Time::now();
	ICP_path.header.frame_id = "odom";
/**** test matlab****/
	SVD_pose_pub = nh_.advertise<geometry_msgs::Point>("SVD_pose",1,true);
	ICP_pose_pub = nh_.advertise<geometry_msgs::Point>("ICP_pose",1,true);
/*	SVD_pose.header.stamp = ros::Time::now();
	SVD_pose.header.frame_id = "odom";	//pointcloud坐标系
	ICP_pose.header.stamp = ros::Time::now();
	ICP_pose.header.frame_id = "odom";*/

	pose_last.setZero();
	pose_predict.setZero();

	//建立参考距离矩阵
	for(int row=0; row < 5; row++)
	{
		for(int col=0; col < 4; col++)
		{
			int row_ = (row + col + 1) % 5;	//回环计算
			ref_dist(row, col) = calc_dist( ref_reflector[row][0], ref_reflector[row][1], ref_reflector[row_][0], ref_reflector[row_][1] );
			printf("ref_dist row:%d, col:%d, value:%f \n",row,col,ref_dist(row,col));	//输出全局反光板矩阵
		}
	}
	return true;
}

/* 反光板匹配：静态距离矩阵匹配 */
/* input：ref_dist、det_dist */
/* output：Table_ID */	 
bool RegisterReflector::regist_ID(int det_num)
{
	int det_col=0; //记录检测距离矩阵的索引
	int det_row=0;

	int row1 = 0; //记录检测矩阵 det(i,0) 在参考矩阵中的位置
	int col1 = 0;
	int ref_col_; //当确定了 det(i,0) 位置之后，对剩余的 det(i,m) 进行依次匹配
	int ref_row_; 
	bool Search_Flag = false; //记录是否匹配成功 只有当 det 中的每个元素均在 ref 中找到对应值，才算匹配成功，否则认为本次扫描匹配失败
	if((det_num > 5) || (det_num < 3)) //判断反光板个数是否符合要求
	{
		return false;
	}
	printf("the det_num is %d\n",det_num);
	printf("ref_dist:row-%ld col-%ld; det_dist:row-%ld col-%ld\n",ref_dist.rows(),ref_dist.cols(),det_dist.rows(),det_dist.cols());
	printf("the Distance_Error is %f\n",Distance_Error);
	printf("\n");
	Table_ID.resize(det_num);
	for(int ref_col=0; ref_col < ref_dist.cols(); ref_col++)
	{
		// step1：参考矩阵 从上到下 从左到右 进行检测矩阵的(0，0)匹配
		for(int ref_row=0; ref_row < ref_dist.rows(); ref_row++)
		{
			det_row = 0; //回到这个位置，相当于对 det(0,0)重新配对
			det_col = 0;
			Table_ID.setZero();
		//	if( (ref_dist(ref_row, ref_col) <= det_dist(det_row, det_col)) && (fabs(ref_dist(ref_row, ref_col)-det_dist(det_row, det_col)) < Distance_Error))
			if(  fabs(ref_dist(ref_row, ref_col)-det_dist(det_row, det_col)) < Distance_Error )
			{
				/*******/
				printf("1\n");
				printf("ref_row: %d	ref_col: %d\n",ref_row,ref_col);
				printf("det_row: %d	det_col: %d\n",det_row,det_col);
				/********/
				//每次失败返回后重新对 def_row、def_col重新赋值，相当于检测矩阵的重新匹配
				row1 = ref_row;
				col1 = ref_col;
				ref_row_ = row1;
				// ref_row、ref_col的值要记录，以便（0，0）错误匹配后 从记录位置继续进行检索
				// step2：从匹配成功的地方 参考矩阵依次向右（只可能出现在右边）遍历匹配检测矩阵第一行剩余元素（只有行的方向进行匹配）
				for( ref_col_=ref_col+1, det_col=det_col+1; ref_col_ < ref_dist.cols(); ref_col_++)
				{
			//		if( (det_col < det_dist.cols()) &&  (ref_dist(ref_row_, ref_col_) <= det_dist(det_row, det_col)) && (fabs(ref_dist(ref_row_, ref_col_)-det_dist(det_row, det_col)) < Distance_Error) )
					if( (det_col < det_dist.cols())  && (fabs(ref_dist(ref_row_, ref_col_)-det_dist(det_row, det_col)) < Distance_Error) )
					{
						/********/
						printf("1_col_match\n");
						printf("ref_col_: %d	det_col: %d\n",ref_col_,det_col);
						/*******/
						// 进行检测矩阵下一个元素的匹配
						det_col++;
					}
				}
				if(det_col == det_dist.cols())	//说明检测矩阵这一行都已经匹配成功
				{
					Table_ID(det_row) = row1;	//记录索引（行号匹配）并为下一次匹配进行索引更新
					det_row++;
					det_col = 0;
					ref_row_ = (row1+col1+1)%5;
					ref_col_ = 0;
					for( ref_col_ = ref_col_; (det_row < det_dist.rows()) && (ref_col_ < ref_dist.cols()); ref_col_++)
					{	
				//		if( (ref_dist(ref_row_, ref_col_) <= det_dist(det_row, det_col)) && (fabs(ref_dist(ref_row_, ref_col_)-det_dist(det_row, det_col)) < Distance_Error))	//反光板首元素匹配上
						if( fabs(ref_dist(ref_row_, ref_col_)-det_dist(det_row, det_col)) < Distance_Error)
						{
							printf("2\n");
							printf("ref_row_: %d	ref_col_: %d\n",ref_row_,ref_col_);
							printf("det_row: %d	det_col: %d\n",det_row,det_col);
							row1 = ref_row_;
							col1 = ref_col_;
							for( ref_col_=ref_col_+1, det_col =1; ref_col_ < ref_dist.cols(); ref_col_++)
							{
						//		if((det_col < det_dist.cols()) && (ref_dist(ref_row_, ref_col_) <= det_dist(det_row, det_col))  && (fabs(ref_dist(ref_row_, ref_col_)-det_dist(det_row, det_col)) < Distance_Error) )
								if((det_col < det_dist.cols())  && (fabs(ref_dist(ref_row_, ref_col_)-det_dist(det_row, det_col)) < Distance_Error) )
								{
									/********/
									printf("2_col_match\n");
									printf("ref_col_: %d	det_col: %d\n",ref_col_,det_col);
									/*******/
									det_col++;
								}
							}
							if(det_col == det_dist.cols())	//改行匹配成功（反光板匹配成功），为下一次匹配进行索引更新
							{
								Table_ID(det_row) = row1;
								det_row++;
								det_col = 0;
								ref_row_ = (row1+col1+1)%5;
								ref_col_ = -1;	//因为在匹配下一块反光板时仍然是在for循环体内，执行完循环块之后会进行ref_col_++, 所以ref_col_=-1,保证下一次从[ref_row_,0]处开始检索
							}
						}
					}
					if(det_row == det_dist.rows()) //所有检测反光板均完成匹配
					{
						Search_Flag = true;
					}
				}
			}
			if(Search_Flag == true) //如果匹配都成功，则直接返回，否则重新进行匹配 det(0,0)
				return Search_Flag;
			else
				continue;
		}
	}
	//如果矩阵遍历完仍然没有找到，则匹配失败
	return Search_Flag;
}

/* 反光板匹配：动态预估方法 */
/* input：pose_predict、det_reflector、 ref_reflector */
/* output：Table_ID */
bool RegisterReflector::regist_ID(int det_num, const reflector_locate::reflector_msgs &msg)
{
	/* 间隔基本在0.2s */
	// time_current = ros::Time::now();
	// predict_period_ = (time_current - time_last).toSec();
	// time_last = time_current;
	// std::cout << "predict_period_ is	"  << predict_period_ << std::endl;

	/* 位姿预估 */
	pose_predict_func();
	/* 根据预估的位姿构建 预估反光板列表 */
	Eigen::MatrixXd ref_list(5, 2);
	Eigen::MatrixXd det_list(det_num, 2);

	double x_ = pose_predict(0);
	double y_ = pose_predict(1);
	double theta_ = pose_predict(2);	//  0——360

	if((det_num > 5) || (det_num < 3)) //判断反光板个数是否符合要求
	{
		return false;
	}
	double tt_;		//tt_用于调试，看预估的角度
	//构建预估列表
	for(int i = 0; i < 5; i++)
	{
		//distance
		ref_list(i, 0) = sqrt( pow((ref_reflector[i][0] - x_),2) + pow((ref_reflector[i][1] - y_),2) ); 
		double dd_ = ref_list(i, 0);
		//theta
		double theta_ref = atan2((ref_reflector[i][1] - y_) , (ref_reflector[i][0] - x_)); 
		theta_ref = (theta_ref > 0.0) ? theta_ref : (theta_ref + 2*Pi) ;	// 0——360
		ref_list(i, 1) = ( (theta_ref - theta_)>0.0 ? (theta_ref-theta_) : ((theta_ref-theta_) + 2*Pi));
		 tt_ = ref_list(i, 1);	// 0——360
		 tt_ = ref_list(i, 1)*(180/Pi);
	}
	//构建检测列表
	for(int i = 0; i < det_num; i++)
	{
		det_list(i, 0) = msg.ref_pose[i].x;
		double dd_ = det_list(i, 0);
	//	det_list(i, 1) = msg.ref_pose[i].y;
    	det_list(i, 1) = (msg.ref_pose[i].y > 0) ? msg.ref_pose[i].y : (msg.ref_pose[i].y+2*Pi) ;
		 tt_ = det_list(i, 1);
		 tt_ = det_list(i, 1)*(180/Pi);
	}
	/* 预估列表与检测列表进行匹配 */
	int det_row ; //检测列表索引
	int ref_row ; //参考列表索引
	int row_;
	bool Search_Flag = false;
	Table_ID.resize(det_num);
	for(ref_row = 0; ref_row < 5; ref_row++)
	{
		det_row = 0;	//每次回到这个地方就相当于重新进行对det_list[0]配对，同时要对Table_ID重置
		Table_ID.setZero(); 
//		tt_ =  ref_list(ref_row, 1);
//		tt_ = det_list(det_row, 1);
		if((fabs(ref_list(ref_row, 0)-det_list(det_row, 0))<Pre_dis_error) && (fabs(ref_list(ref_row, 1)-det_list(det_row, 1))<Pre_ang_error))
		{
			Table_ID(det_row) = ref_row;
			det_row ++;	//检测列表位置更新
			for(row_ = ref_row+1; row_ < ref_row+5; row_++) //剩余四个进行匹配 %5取余
			{
				if((det_row < det_num) && (fabs(ref_list(row_%5, 0)-det_list(det_row, 0))<Pre_dis_error) && (fabs(ref_list(row_%5, 1)-det_list(det_row, 1))<Pre_ang_error))
				{
					Table_ID(det_row) = row_%5;
					det_row++;
				}
				if(det_row == det_num)	//匹配成功条件
				{
					Search_Flag = true;
					return Search_Flag;
				}
			}
		}
		if(Search_Flag == true)
			return Search_Flag;
		else
			continue;
	}
	return Search_Flag;
}

/* 预测AGv下一时刻位姿 */
/* input：pose_last 、 v_w_ 、时间间隔（loop_rate）:200ms*/
/* output:pose_predict     */
void  RegisterReflector::pose_predict_func()
{
	double x_ = v_w_.linear.x * predict_period * cos(pose_last(2));
	double y_ = v_w_.linear.x * predict_period * sin(pose_last(2));
	double theta_ = v_w_.angular.z * predict_period;
	pose_predict(0) = pose_last(0) + x_;
	pose_predict(1) = pose_last(1) + y_;
	pose_predict(2) = pose_last(2) + theta_;
	
	pose_predict(2) = ( (pose_predict(2) < 0.0) ?  (pose_predict(2) + 2*Pi) : pose_predict(2) );	
	pose_predict(2) = ( (pose_predict(2) > 2*Pi) ?  (pose_predict(2) - 2*Pi) : pose_predict(2) );	
}

/* 构建ICP的局部坐标矩阵 */
/* flag :0 ref_last */
/* flag :1 ref_current */
void RegisterReflector::set_ICP_list(int num, const reflector_locate::reflector_msgs &msg, int flag)
{
	Eigen::MatrixXd tmp;
	tmp.resize(2, num);
	for(int i=0; i<num; i++)
	{
		tmp(0, i) = msg.ref_pose[i].x * cos(msg.ref_pose[i].y);	//x
		tmp(1, i) = msg.ref_pose[i].x * sin(msg.ref_pose[i].y);	//y
	}
	if(flag == 0)	//last
	{
		ref_last.setZero();
		ref_last.resize(2, num);
		ref_last = tmp;
		//在flag = 0，代表进入初始化（或者失败后的初始化），要对Table_ID_last重新赋值
		Table_ID_last.resize(num);	
		Table_ID_last = Table_ID;
	}
	else	//current
	{
		ref_current.setZero();
		ref_current.resize(2, num);
		ref_current = tmp;
	}
}

/* 需要的信息 */
/* msg：检测的反光板信息 distance,theta 可以直接用生成的ref_dist? no,那是相对距离矩阵，没用呀 */
/* 检测反光板与参考反光板之间的对应列表，以便于确定检测反光板地全局坐标 */
/* 构建关系矩阵 AX=B SVD方法分解进行 超定最小二乘 线性 方程的求解 */
bool RegisterReflector::compute_pose_linearSVD(const reflector_locate::reflector_msgs &msg)	
{
	int det_num = msg.id;
	int i, ref_id, ref_endid;	//依次：循环变量、检测对应的参考id、检测最后一个对应的参考id （因为拿前n-1个与第n个作差构建线性方程）
	Eigen::Vector2d pose_;

	//A*pose_linearSVD = B
	Eigen::Vector3d pose_linearSVD;
	Eigen::MatrixXd A;
	Eigen::VectorXd B;

	A.resize(det_num-1, 2);
	B.resize(det_num-1);
	ref_endid = Table_ID(det_num-1);	
	//构造A B矩阵
	for(i = 0; i < det_num-1; i++)
	{
		ref_id = Table_ID(i);	//第i个检测反光板对应的第ref_id个预存反光板
		
		A(i, 0) = (ref_reflector[ref_endid][0] - ref_reflector[ref_id][0]) * 2;
		A(i, 1) = (ref_reflector[ref_endid][1] - ref_reflector[ref_id][1]) * 2;
		double x_ = pow(ref_reflector[ref_endid][0], 2) - pow(ref_reflector[ref_id][0], 2);
		double y_ = pow(ref_reflector[ref_endid][1], 2) - pow(ref_reflector[ref_id][1], 2);
		double d_ = pow(msg.ref_pose[i].x, 2) - pow(msg.ref_pose[det_num-1].x, 2);
		B(i) = x_ + y_ + d_; 
	}
	//SVD求解
	pose_ = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
	pose_linearSVD(0) = pose_(0);
	pose_linearSVD(1) = pose_(1);
	//计算航向角
	double theta_ = 0.0;
	double temp_theta_min = 0.0;
	double temp_theta_max = 0.0;
	double theta_sum = 0.0;
	double theta_sum_ = 0.0;
	for(i = 0; i < det_num; i++)
	{
		ref_id = Table_ID(i);
//		double theta_i = (msg.ref_pose[i].y) * (180 / Pi);	//rad 雷达检测
		double theta_i = msg.ref_pose[i].y;
		double theta_ref = atan2((ref_reflector[ref_id][1] - pose_(1)) , (ref_reflector[ref_id][0] - pose_(0))); 
		theta_i = (theta_i > 0.0) ? theta_i : (theta_i + 2*Pi); 	// 0~180,-180~0 ——>  0~360
		theta_ref = (theta_ref > 0.0) ? theta_ref : (theta_ref + 2*Pi) ; 	//	0～360
		printf("num is: %d\n", i);
		printf("theta_i: %f	theta_ref: %f	correspond theta is: %f\n",theta_i*180/Pi, theta_ref*180/Pi, (theta_ref - theta_i)*180/Pi );
/*		if( fabs(theta_ref - theta_i) < 0.0872665)	//判断是否处于360度附近(正负5度)，由于0/360是重合的，但是平均就成了180
			theta_ += (theta_ref - theta_i);
		else 
			theta_ += ((theta_ref - theta_i) > 0.0 ? (theta_ref - theta_i) : (theta_ref - theta_i + 2*Pi) );	*/
		theta_ = theta_ref - theta_i;
		theta_sum_ += theta_;

		theta_ =  (theta_ > 0.0 ? theta_ : (theta_ + 2*Pi) );
		theta_sum += theta_;
		if(i == 0)
		{
			temp_theta_max = theta_;
			temp_theta_min = theta_;
		}
		else
		{
			if( temp_theta_max < theta_)
				temp_theta_max = theta_;
			if( temp_theta_min > theta_ )
				temp_theta_min = theta_;
		}
	}
	if((temp_theta_max-temp_theta_min) > 5.7596)
		theta_ = theta_sum_/det_num;	//认为就是360度
	else
		theta_ = theta_sum/ det_num ;
	pose_linearSVD(2) = ( (theta_ > 0.0) ? theta_ : (theta_ + 2*Pi) );	//将角度范围转移到（0，360）区间之内 这里的角度不会越界360度
//	pose_linearSVD(2) = theta_ * (180/pi);
	std::cout << "pose_SVD is" << "\n" << pose_linearSVD << std::endl;

	pose_last = pose_linearSVD;

	SVD_pose.x = pose_linearSVD(0);
	SVD_pose.y = pose_linearSVD(1);
//	SVD_pose.header.stamp=ros::Time::now();
	SVD_pose_pub.publish(SVD_pose);

	pub_path(pose_linearSVD, SVD_path, SVD_path_pub);	//打印路径
}

/* 进行ICP计算，前后的反光板个数必须一致 （与理论公式有关）*/
bool RegisterReflector::compute_pose_ICP()
{
	Eigen::Vector3d pose_ICP;	//计算当前的ICP位姿
	Eigen::MatrixXd _last_;
	Eigen::MatrixXd _current_;
	/* 根据table_ID_last与table_ID 建立对应关系表 （个数一致）*/
	vector<vector<int> > correspond_table;	//二维数据，每一行记录对应点的ID
	vector<int> tmp;
	for(int i=0; i<Table_ID_last.size(); i++)
	{
		tmp.clear();	//每次要清空临时向量
		for(int j=0; j<Table_ID.size(); j++)
		{
			if(Table_ID_last(i) == Table_ID(j))
			{
				tmp.push_back(i);
				tmp.push_back(j);
				correspond_table.push_back(tmp);
			}
		}
	}
	int size_ = correspond_table.size();
	if(size_ < 2)
	{
		std::cout << "correspond num is too less" << std::endl;
		return false;
	}
	_last_.resize(2, size_);	//对ICP的两个矩阵赋值
	_current_.resize(2, size_);
	icp_achieve pose_icp_solve(size_);	//创建ICP解法的类
	/* 对ICP中的两个待匹配矩阵进行赋值 ,并使用类的方法进行求解*/
	for(int i=0; i<size_; i++)
	{
		int last_ID = correspond_table[i][0];
		int current_ID = correspond_table[i][1];
		_last_(0, i) = ref_last(0, last_ID);	
		_last_(1, i) = ref_last(1, last_ID);
		_current_(0, i) = ref_current(0, current_ID);
		_current_(1, i) = ref_current(1, current_ID);
	}
	/*  ICP类计算实现 */
	pose_icp_solve.last_ = _last_;
	pose_icp_solve.current_ = _current_;
	pose_icp_solve.compute_rigid_transform();
	//分别求出delta x 、 delta y、theta
/*  错误：下面的R、t关系是两帧点云坐标之间的转换，而不是机器人位姿的变换
	pose_ICP.head(2) = pose_icp_solve.R * pose_last.head(2) + pose_icp_solve.T; 
*/
	double x_ = pose_icp_solve.T(0);
	double y_ = pose_icp_solve.T(1);
	double theta_ = atan2(pose_icp_solve.R(0,1), pose_icp_solve.R(0,0));
	theta_ = (theta_ > 0.0) ? theta_ : (theta_ + 2*Pi) ;
	/* 局部位姿变换——>全局位姿变换	理论推导：汇报 */
	double c,s;
	Eigen::Matrix3d transform_matrix;
	Eigen::Vector3d d_point_odom;	//局部位姿差
	c = cos(pose_last(2));	//前一时刻的航向角
    s = sin(pose_last(2));
    transform_matrix << c,-s,0,
                       						    s, c,0,
                      							0, 0,1;
    d_point_odom << x_, y_, theta_;
    pose_ICP = pose_last + transform_matrix*d_point_odom;	//局部位姿查转换到全局坐标系下，方可进行相邻时刻全局位姿的推算
    /*
    	两个正角度相加可能会超过360，但不会为负
    	两个正角度相减可能为负，但不会超过360
    */
	pose_ICP(2) = (pose_last(2) > 2*Pi) ? (pose_last(2) - 2*Pi) : pose_last(2); 
	std::cout << "pose_ICP is" << "\n" << pose_ICP << std::endl;

	ref_last = ref_current;	//记录上一时刻反光板局部坐标
	Table_ID_last.resize(Table_ID.size());
	Table_ID_last = Table_ID;	//Table_ID_last与ref_last是对应的

	pose_last = pose_ICP;	//记录上一时刻位姿（预估位姿）

	ICP_pose.x = pose_ICP(0);
	ICP_pose.y = pose_ICP(1);
//	ICP_pose.header.stamp=ros::Time::now();
	ICP_pose_pub.publish(ICP_pose);

	pub_path(pose_ICP, ICP_path, ICP_path_pub);	//打印路径
}

void RegisterReflector::MoveInfoCallback(const geometry_msgs::Twist &msg)
{
	// 两轮差速
	v_w_.linear.x = msg.linear.x;
	v_w_.angular.z = msg.angular.z;
}

void RegisterReflector::DetReflectorCallback(const reflector_locate::reflector_msgs &msg)
{
	int ref_num = msg.id; //检测到的反光板的个数
	det_dist.resize(ref_num, ref_num-1);
	float d1,th1,d2,th2;
	//建立检测距离矩阵,仅在初始匹配构建
	if(Initial_Flag == false)	//静态初始匹配
	{
		for(int row=0; row < ref_num; row++)	//构建检测矩阵
		{
			d1 = msg.ref_pose[row].x;
			th1 = msg.ref_pose[row].y;
			for(int col=0; col < ref_num-1; col++)
			{
				int row_ = (row + col + 1) % ref_num;
				d2 = msg.ref_pose[row_].x;
				th2 = msg.ref_pose[row_].y;

				det_dist(row, col) = calc_dist(d1, d2, th2-th1);
				printf("det_dist row:%d, col:%d, value:%f \n",row,col,det_dist(row,col));
			}
		}
		if(regist_ID(ref_num))
		{
			std::cout << "initial static regist_ID is" << "\n" << Table_ID << std::endl;	//输出反光板匹配序列
			compute_pose_linearSVD(msg); 

			set_ICP_list(ref_num, msg, set_last);	//对ref_last进行赋值，若flag ==0 ,也将对Table_ID_last赋值（相当于 "初始化 "ref_last、Table_ID_last）
			pub_path(pose_last, ICP_path, ICP_path_pub);	//ICP定位的初始位姿以及定位失败后的位姿	
			Initial_Flag = true;
		}		
		else
		{
			std::cout << "initial static regist failed baba!" << std::endl;
		}
	}
	else	//动态匹配
	{
		if(regist_ID(ref_num, msg))
		{
			std::cout << "dynamic regist_ID is" << "\n" << Table_ID << std::endl;
//			compute_pose_linearSVD(msg);

			set_ICP_list(ref_num, msg, set_current);
			compute_pose_ICP();		//该函数体内对Table_ID_last、 pose_last进行了 "赋值"。动态反光板匹配使用的pose_last是ICP计算的结果
		}		
		else
		{
			Initial_Flag = false;	//当动态定位失败之后，使用静态定位方法重新更新pose_linearSVD
			std::cout << "dynamic regist failed!" << std::endl;
		}
	}	
}

//将路径发布
void RegisterReflector::pub_path(const Eigen::Vector3d &pose, nav_msgs::Path &path, ros::Publisher &_path_pub)
{
	ros::Time current_time;
	geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = pose(0);
    this_pose_stamped.pose.position.y = pose(1);

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(pose(2));
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

	current_time = ros::Time::now();
    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="odom";
    path.poses.push_back(this_pose_stamped);	//nave::mags里面是 geometry_mags::PoseStamped[]
    _path_pub.publish(path);	//所以说每次增加一个poses就发布一次？
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "register_reflector");
	RegisterReflector Register_Reflector;

	Register_Reflector.init();

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
    	ros::spinOnce();
    	loop_rate.sleep();
	}

	return 0;
}
