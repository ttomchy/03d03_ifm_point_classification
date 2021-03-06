#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//关于平面分割的头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/model_types.h>   //分割模型的头文件
#include <pcl/sample_consensus/method_types.h>   //采样一致性的方法
#include <pcl/segmentation/sac_segmentation.h>  //ransac分割法
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

int i = 0;
void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;

    pcl::fromROSMsg (*input, cloud);   //关键的一句数据的转换

    std::vector<int> index;

    pcl::removeNaNFromPointCloud(cloud, cloud, index);

    std::cerr << "PointCloud before filtering: " << cloud.width * cloud.height
              << " data points (" << pcl::getFieldsList (cloud) << ")."<<std::endl;

    pcl::PCDWriter writer;
    std::stringstream ss,ss_filtered;
    ss << "o3d03_origin" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), cloud, false);

    // 进行一个滤波处理
    pcl::VoxelGrid<pcl::PointXYZ> sor;   //实例化滤波
    sor.setInputCloud (cloud.makeShared());     //设置输入的滤波
    sor.setLeafSize (0.016, 0.016, 0.016);   //设置体素网格的大小
    sor.filter (cloud_filtered);      //存储滤波后的点云

    std::cerr << "PointCloud after filtering: " << cloud_filtered.width * cloud_filtered.height
              << " data points (" << pcl::getFieldsList (cloud_filtered) << ")."<<std::endl;


    ss_filtered << "o3d03_cloud_filtered_0016" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss_filtered.str (), cloud_filtered, false);
    i++;



}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/o3d3xx/camera/cloud", 1, cloud_cb);

    //创建ROS的发布节点
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/chy/point", 1);

    // 回调
    ros::spin ();
}