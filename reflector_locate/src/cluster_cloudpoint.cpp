#include "reflector_locate/cluster_cloudpoint.h"

//条件欧式聚类判断条件
//(point_a.intensity - point_b.intensity) < 0.10f强度差小于0.1表示，只有强度相近的点才能聚为一类
//距离平方和小于0.1表示，只有距离相近的点才能聚为一类
//(point_a.intensity + point_b.intensity>500圆柱的点云强度大概为255，强度和大于500为一个约束条件，相邻两点都为圆柱才能聚合在一起。
bool enforceIntensitySimilarity(const pcl::PointXYZI& point_a, const pcl::PointXYZI& point_b, float squared_distance)
{
	if (fabs(point_a.intensity - point_b.intensity) < 0.10f && ((point_a.x - point_b.x)*(point_a.x - point_b.x)
		+ (point_a.y - point_b.y)*(point_a.y - point_b.y) + (point_a.z - point_b.z)*(point_a.z - point_b.z))<0.1
		&& (point_a.intensity + point_b.intensity>500))
		return (true);
	else
		return (false);
}
//计算聚类方向角
double getAngleOfCluster(const pcl::PointCloud<pcl::PointXYZI> &cloud )
{
	double centroid_x=0,centroid_y=0;
	unsigned int k;
	for(k=0; k!=cloud.size(); ++k )
	{
		centroid_x += cloud.points[k].x;
		centroid_y += cloud.points[k].y;
	}
	centroid_x/=k;
	centroid_y/=k;
	return atan2(centroid_y, centroid_x);
}
//按方向角对聚类进行排序（冒泡排序）
void sortClusterClockwise(std::vector< pcl::PointCloud<pcl::PointXYZI> > &input_cloud, std::vector<double> &angleIndex)
{
	unsigned int scale = input_cloud.size();
	for(unsigned int i = 0; i != scale-1; ++i)
	{
		for(unsigned int j=0; j != scale-1-i; ++j)
		{
			double a1,a2;
			a1 = (angleIndex[j]<0.0 ? (angleIndex[j]+2*pi) : angleIndex[j] );
			a2 = (angleIndex[j+1]<0.0 ? (angleIndex[j+1]+2*pi) : angleIndex[j+1] );
			if(a1<a2)
			{
				pcl::PointCloud<pcl::PointXYZI> temp;
				temp = input_cloud[j];
				input_cloud[j] = input_cloud[j+1];
				input_cloud[j+1] = temp;

				double temp_;
				temp_ = angleIndex[j];
				angleIndex[j] = angleIndex[j+1];
				angleIndex[j+1] = temp_;
			}
		}
	}
	std::cout << "after sorting:" << std::endl; 
	//输出排序后的方向角
	for(unsigned int i = 0; i != input_cloud.size(); ++i)
	{
		std::cout << "the angle of " << i << "is" << angleIndex[i] << std::endl;
	}
}
//将聚类结果叠加到cloud_final
void addClusterCloud(const std::vector< pcl::PointCloud<pcl::PointXYZI> > &input_cloud, pcl::PointCloud<pcl::PointXYZI> &output_cloud)
{
	for(unsigned int i = 0; i != input_cloud.size(); ++i)
	{
		output_cloud += input_cloud[i];
	}
}

ClusterCloudpoint::ClusterCloudpoint()
{
    ROS_INFO("ClusterPoint Node Init");
}

ClusterCloudpoint::~ClusterCloudpoint()
{
    ros::shutdown();
}

void ClusterCloudpoint::init()
{
    clusterPointPub = nh.advertise<sensor_msgs::PointCloud2>("clusterPoint",10);
    pointcloudSub = nh.subscribe("/rslidar_points", 10, &ClusterCloudpoint::pointCloudCallBack, this);
}

 void ClusterCloudpoint::pointCloudCallBack(const sensor_msgs::PointCloud2ConstPtr& msg)
 {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);//输入点云 指针
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
	 pcl::fromROSMsg(*msg, *cloud_in);	//	将点云数据转换为pcl格式进行解析

    clock_t startTime, endTime_filter,endTime_cluster;;
	startTime = clock();//计时开始     

    //原点云显示
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> raw_cloud(cloud_in, "intensity");

    //直通滤波
     pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(cloud_in);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-2, 5.0);
	pass.filter(*cloud_filtered);

	pass.setInputCloud(cloud_filtered);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-50.0, 60.0);
	pass.filter(*cloud_filtered);

	pass.setInputCloud(cloud_filtered);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-15, 15);
	pass.filter(*cloud_filtered);

    endTime_filter = clock();
    std::cout << "直通滤波时间: " << (double)(endTime_filter - startTime) / CLOCKS_PER_SEC << "s" << endl;
	std::cerr << "直通滤波后点云: " << std::endl;

    //条件欧式聚类开始
	pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters);
	pcl::ConditionalEuclideanClustering<pcl::PointXYZI> cec(true);
	cec.setInputCloud(cloud_filtered);
	cec.setConditionFunction(&enforceIntensitySimilarity);
	cec.setClusterTolerance(0.2);
	cec.setMinClusterSize(20);	//限制最小的点云集的数量
	cec.setMaxClusterSize(100000);
	cec.segment(*clusters);
	//条件欧式聚类结束	
   
   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr add_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	std::vector< pcl::PointCloud<pcl::PointXYZI> > add_cloud_;
	std::vector<double> angleIndex;
	std::cout << "clusters->size(): " << clusters->size() << " n" << endl;

    //对聚类结果进一步提取
	int m = 0;
	std::cout<<"the number of cluster is: " << clusters->size() << std::endl;	//聚类的个数大概是22个
	//针对聚类的高度（圆柱体高度是0.75，作进一步的筛选处理）
	for (unsigned int i = 0; i < clusters->size(); ++i)
	{
		//将属于一个聚类的点云放入临时的点云集 add_cloud 中
		for (unsigned int j = 0; j < (*clusters)[i].indices.size(); ++j)
		{
			cloud_filtered->points[(*clusters)[i].indices[j]].intensity = i;
			add_cloud->push_back(cloud_filtered->points[(*clusters)[i].indices[j]]);
		}
		pcl::PointXYZI minPt, maxPt;
		pcl::getMinMax3D(*add_cloud, minPt, maxPt);
		std::string detectstring = std::to_string(m);
		//进一步对聚类进行筛选（形状特征）
		//为符合圆柱尺寸的点云画上包围框（高度范围0.5 ~ 1.0的聚类认为是圆柱的）
		// if ((maxPt.z - minPt.z) <1 && (maxPt.z - minPt.z) >0.5 && (maxPt.y - minPt.y)>0.1 && (maxPt.y - minPt.y) <0.5 && (maxPt.x - minPt.x) <0.7 && (maxPt.x - minPt.x)>0.1)
		if( (maxPt.z-minPt.z)>0.5 &&(maxPt.z-minPt.z)<1.0)
		{
			viewer->addCube(minPt.x, maxPt.x, minPt.y, maxPt.y, minPt.z, maxPt.z, 1, 1, 1, detectstring);
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, detectstring);
			m++;

			//计算当前圆柱体簇的位置（只需要角度？根据角度来判断是哪个反光板）
			double clusterAngle = getAngleOfCluster(*add_cloud);
			std::cout << "the angle of cluster  index: " << i << " is :" << clusterAngle*180/pi << std::endl;	

			//将所有符合圆柱体特征的点云放入点云vector中
			//并存储点云集的方向角信息
			add_cloud_.push_back(*add_cloud);
			angleIndex.push_back(clusterAngle);
			add_cloud->clear();
		}
	}
	//将点云集按角度顺时针方向进行排序
	sortClusterClockwise(add_cloud_, angleIndex);
	//排序后的点云集vector叠加到 cloud_final上
	addClusterCloud(add_cloud_, *cloud_final);
	// 发布聚类之后的点云信息 ，反光被提取部分订阅该点云信息
	sensor_msgs::PointCloud2 output_cloud;
	pcl::toROSMsg(*cloud_final, output_cloud);
	output_cloud.header.frame_id = "rslidar";	//注意要给出点云所在的坐标系
    clusterPointPub.publish(output_cloud);

	endTime_cluster = clock();
	std::cout << "条件欧式聚类时间: " << (double)(endTime_cluster - endTime_filter) / CLOCKS_PER_SEC << "s" << endl;
	std::cerr << "地面点滤除条件欧式聚类后点云: " << std::endl;
    std::cerr << *cloud_final << std::endl;

    viewer0->addPointCloud(cloud_in, raw_cloud, "cloud_in");
	viewer0->addCoordinateSystem();
	viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_in");
    //聚类点云显示
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> inten_color(cloud_final, "intensity");
	viewer->addPointCloud(cloud_final, inten_color, "cloud");
	viewer->addCoordinateSystem();
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	
 }

int main(int argc, char** argv)
{
	ros::init (argc, argv, "Cluster");
    ClusterCloudpoint ClusterCloudpoint_;
    ClusterCloudpoint_.init();
    ros::Rate loop_rate(10);
	while (ros::ok())
	{ 
		viewer0->spinOnce();
		viewer->spinOnce();
        ros::spinOnce();
        loop_rate.sleep();
	}
	return 0;
}