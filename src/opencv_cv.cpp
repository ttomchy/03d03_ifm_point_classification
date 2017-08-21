#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <fstream>
using namespace std;
int
main (int argc, char** argv) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_dst(new pcl::PointCloud<pcl::PointXYZI>);
    int j_num_wall =200;
    std::string ss1;

   while(j_num_wall<302) {

       char szName[100] = {'\0'};
      // sprintf(szName, "/home/laptop2/work_space/intern_ws/o3d/test_ws/dataset/training/wall/clouser_wall%d.pcd", j_num_wall); //格式化输出文件名
       sprintf(szName, "/home/laptop2/work_space/intern_ws/o3d/test_ws/dataset/training/not_wall/clouser_not_wall%d.pcd", j_num_wall); //格式化输出文件名

       // Fill in the cloud data
       pcl::PCDReader reader;
       // Replace the path below with the path where you saved your file
       reader.read(szName, *cloud); // Remember to download the file first!


       std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
                 << " data points (" << pcl::getFieldsList(*cloud) << ").";

       std::cerr << "The number of the  j_num_wall is :" <<j_num_wall<< std::endl;

       //pcl::PointCloud<pcl::PointXYZI> temp;
       for (size_t i = 0; i < cloud->points.size(); ++i) {

           std::cout << cloud->points[i].x << " "
                     << cloud->points[i].y << " "
                     << cloud->points[i].z << " " << 1200 << std::endl;
       }
      j_num_wall++;

   }





    return (0);
}
