#include "reflector_locate/reflector_extract.h"

#define pi 3.1415926
#define Lidar_error 0.13

ReflectorExtract::ReflectorExtract():nh_priv_("~")
{
  ROS_INFO("Locate Node Init");
  ROS_INFO("Locate Init");
  ROS_ASSERT(init());
}

ReflectorExtract::~ReflectorExtract()
{
   ros::shutdown();
}

bool ReflectorExtract::init()
{
	reflector_pub_ = nh_.advertise<reflector_locate::reflector_msgs>("reflector_pose",10);
  // *** reflector_msgs.msg
  // *** uint32 id                      反光板的序号
  // *** geometry_msgs/Point[] ref_pose 反光板的位置（距离、角度）

  // initialize subscribers
//	pointcloud_sub_  = nh_.subscribe("/rslidar_points", 10, &ReflectorExtract::pointCloudCallBack, this);
	pointcloud_sub_  = nh_.subscribe("clusterPoint", 10, &ReflectorExtract::clusterPointCallback, this);
	return true;
}

/*从聚类后的点云中提取反光板信息*/
/*聚类中已经把反光板的点云都提取出来，并且对于不同的反光被已经
分别赋予了不同的intensity:0,1,2,...便于区分*/
 void ReflectorExtract::clusterPointCallback(const sensor_msgs::PointCloud2 &msg)
 {
	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::fromROSMsg(msg, cloud);	//	将点云数据转换为pcl格式进行解析
	geometry_msgs::Point point;
	reflector_locate::reflector_msgs ref_msgs;	//待发布的反光板信息

	if(!cloud.size())
	{
		std::cout << "Error : the clusterPoint is empty!" << std::endl;
		return ;
	}
	float distance;	
	float angle;	//反光板的距离、角度信息
	unsigned int ref_id = 0; //记录扫描的反光板序号
	auto current_intensity = cloud.points[0].intensity;
	double px=0,py=0;
	unsigned int count_point=0;
	for(unsigned int i=0; i<cloud.size(); ++i)
	{
			while( (i<cloud.size()) && (cloud.points[i].intensity == current_intensity) )
			{
				px+=cloud.points[i].x;
				py+=cloud.points[i].y;
				count_point++;
				i++;
			}
			px /= count_point;
			py /= count_point;
			distance = sqrt(px*px +py*py);
			angle = atan2(py,px);
			point.x = distance;
			point.y = angle;
			point.z = ref_id;	//检测到的第几块反光板（0，1，...）
			//angle = angle*180/pi;
			std::cout << "id: " << ref_id << " " << "angle: " << angle*180/pi << "distance: " << distance << std::endl;
			std::cout <<"Lidar_system_error:" << Lidar_error << std::endl;
			ref_msgs.ref_pose.push_back(point);
			ref_id++;

			if(i<cloud.size())
			{
				px = cloud.points[i].x;
				py = cloud.points[i].y;
				count_point =1;
				current_intensity = cloud.points[i].intensity;
			}
	}
	ref_msgs.id = ref_id;	//反光板总数
	reflector_pub_.publish(ref_msgs);
 }

/*
//读取原始的点云信息，根据强度信息从中提取反光板
void ReflectorExtract::pointCloudCallBack(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::fromROSMsg(*msg, cloud);	//	将点云数据转换为pcl格式进行解析
	geometry_msgs::Point point;
	reflector_locate::reflector_msgs ref_msgs;	//待发布的反光板信息

	float distance;	
	float angle;	//反光板的距离、角度信息

	int ref_id = 0; //记录扫描的反光板序号
	int n = cloud.points.size();
	printf("new frame coming!\n");
	//只提取第16通道的雷达扫描信息，即相当于在 +1° 的检测信息 
	for(int i = ceil(n*15/16); i < n; i++)	//向上取整
	{
		distance = 0.0;
		angle = 0.0;
		if(cloud.points[i].intensity ==255)	//经测试，反光板返回为255
		{
			float x = 0.0;
			float y = 0.0;
			int num = 0;	//记录同一块反光板上返回的激光点数 分辨率 0.18°
			while(cloud.points[i].intensity ==255)	//找出反光板上扫描的连续值并平均处理
			{
				x = cloud.points[i].x;
				y = cloud.points[i].y;
				distance += sqrt(x*x + y*y);
				angle += atan2(y,x);
				num++;
				i++;
	//			printf("the index is: %d, angle: %f, distance: %lf, intensity: %f\n",i,angle*180/pi,distance);
			}
	//		printf("num is: %d\n",num);
			if(num>4)	//4 一个反光板返回的激光点数（也可以防止误检反光板的情况）
			{
				//std::cout << "num: " << num << std::endl;
				distance = ( (distance/num - Lidar_error)>0.0 ? (distance/num - Lidar_error) : distance/num );
				//angle = angle/num* (180/pi);
		//		distance = distance/num;
				angle = angle/num;
				point.x = distance;
				point.y = angle;
				point.z = ref_id;	//检测到的第几块反光板（0，1，...）
				//angle = angle*180/pi;
				std::cout << "id: " << ref_id << " " << "angle: " << angle*180/pi << "distance: " << distance << std::endl;
				std::cout <<"Lidar_system_error:" << Lidar_error << std::endl;
				ref_msgs.ref_pose.push_back(point);
				ref_id++;
			}
	//		printf("this over,next!\n");
		}
	}
	ref_msgs.id = ref_id;	//反光板总数
	reflector_pub_.publish(ref_msgs);
}
*/
int main(int argc, char* argv[])
{
	ros::init(argc, argv, "reflector_extract");
	ReflectorExtract reflector_extract_;
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

