#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <thread>
#include <mutex>
#include <deque>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iomanip>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include "../include/features.h"


using namespace std;


template<typename T>
bool swap_if_gt(T& a, T& b) {
    if (a < b) {
        std::swap(a, b);
        return true;
    }
    return false;
}
using namespace Eigen;

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZI PclPoint;

deque< pcl::PointCloud<PointT>> cloud_vector;
std::mutex m_mux;//全局互斥锁

int j_num_wall = 0;
int j_num_not_wall = 0;




void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input) {


    cerr<<"now it runs in the call back function !"<<endl;
    // All the objects needed
    pcl::PCDReader reader;
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PCDWriter writer;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);


    // Read in the cloud data
    //  reader.read ("o3d03_origin0.pcd", *cloud);
    //  std::vector<int> index;
    //  pcl::removeNaNFromPointCloud(*cloud, *cloud, index);

    pcl::fromROSMsg (*input, *cloud);   //关键的一句数据的转换

    std::vector<int> index;

    pcl::removeNaNFromPointCloud(*cloud, *cloud, index);

    std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;




    //create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(25);
    sor.setStddevMulThresh(2.0);
    sor.filter(*cloud_filtered);
    std::cout<<"cloud after filtering:"<<std::endl;




    // std::cerr<<*cloud_filtered<<std::endl;

    //  writer.write<pcl::PointXYZ>("split_map3d_inliers.pcd",*cloud_filtered,false);
    //  std::cout<<"the size of the in-lier points is:"<<cloud_filtered->points.size ()<<std::endl;

    /*
    sor.setNegative(true);
    sor.filter(*cloud_filtered);
    writer.write<pcl::PointXYZ>("split_map3d_outliers.pcd",*cloud_filtered,false);
    std::cout<<"the size of the out-lier points is:"<<cloud_filtered->points.size ()<<std::endl;
  */
    // Estimate point normals

    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.05);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
    // std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    // Write the planar inliers to disk
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_plane);
    std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    // writer.write ("./dataset/training/diff_scale/target/plane.pcd", *cloud_plane, false);


    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);

    pcl::PointCloud<PointT>::Ptr non_cloud_plane (new pcl::PointCloud<PointT> ());
    extract.filter (*non_cloud_plane);
    //  std::cerr << "PointCloud representing the non_cloud_plane planar component: " << non_cloud_plane->points.size () << " data points." << std::endl;
    // writer.write ("no-table_scene_mug_stereo_textured_plane.pcd", *cloud_filtered2, false);

   // *cloud=*non_cloud_plane;
    std::cerr << "  the number of the  ground point is : " << non_cloud_plane->points.size () << " data points." << std::endl;


    *cloud_filtered=*non_cloud_plane;

    writer.write<pcl::PointXYZ> ("origin.pcd", *cloud_filtered,false);


    std::cerr << "PointCloud before filtering: " << cloud_filtered->width * cloud_filtered->height
              << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")."<<endl;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor2;
    sor2.setInputCloud (cloud_filtered);
    sor2.setLeafSize (0.03f, 0.03f, 0.03f);
    sor2.filter (*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
              << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")."<<endl;

    writer.write<pcl::PointXYZ> ("dst.pcd", *cloud_filtered,false);











    std::unique_lock<std::mutex> lck(m_mux);
    cloud_vector.push_back(*cloud_filtered);
    lck.unlock();

    ROS_WARN("PRODUCE ONE PRODUCT");
    cerr<<"The size of the cloud_vector is :"<<cloud_vector.size()<<endl;


}






void EigenvalueBasedDescriptor( pcl::PointCloud<PclPoint> & segment,float local_density, float lable){

    vector<double> feature_vec;

    PclPoint minPt, maxPt;

    //获取坐标极值
    pcl::getMinMax3D(segment, minPt, maxPt);
    feature_vec.push_back(maxPt.z-minPt.z);//delta z

    float local_density_tmp=local_density;
    feature_vec.push_back(local_density_tmp);//local point density


    int kNPoints=segment.points.size();//get the number of the points

    std::vector<std::vector<float>> vec;
    std::vector<float> v1;

    for (int i = 0; i < kNPoints; ++i) {
        v1.push_back(segment.points[i].x);
        v1.push_back(segment.points[i].y);
        v1.push_back(segment.points[i].z);
        vec.push_back(v1);
        v1.clear();
    }

    const int rows{kNPoints}, cols{3};

    std::vector<float> vec_;
    for (int i = 0; i < rows; ++i) {
        vec_.insert(vec_.begin() + i * cols, vec[i].begin(), vec[i].end());
    }
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> m(vec_.data(), rows, cols);

    const int nsamples = rows;

    Eigen::MatrixXf mean = m.colwise().mean();

    Eigen::MatrixXf tmp(rows, cols);
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            tmp(y, x) = m(y, x) - mean(0, x);
        }
    }

    Eigen::MatrixXf covar = (tmp.adjoint() * tmp) / float(nsamples - 1);
    //std::cout << "print covariance matrix: " << std::endl << covar << std::endl;
    EigenSolver<Matrix3f> es(covar);
    Matrix3f D = es.pseudoEigenvalueMatrix();

    //std::cout<<"The eigenvalue is :"<< std::endl;
    //std::cout<<D<<std::endl;

    double e1 = D(0, 0);
    double e2 = D(1, 1);
    double e3 = D(2, 2);

    // Sort eigenvalues from smallest to largest.
    swap_if_gt(e1, e2);
    swap_if_gt(e1, e3);
    swap_if_gt(e2, e3);
    //std::cout<<"The eigenvalue is e1:"<<  e1<< std::endl;
    //std::cout<<"The eigenvalue is e2:"<<  e2<< std::endl;
    //std::cout<<"The eigenvalue is e3:"<<  e3<< std::endl;
    // Normalize eigenvalues.

    double sum_eigenvalues = e1 + e2 + e3;
    feature_vec.push_back(sum_eigenvalues); //sum of eigenvalues.

    e1 = e1 / sum_eigenvalues;
    e2 = e2 / sum_eigenvalues;
    e3 = e3 / sum_eigenvalues;

    // std::cout<<"The value of the gigenvalue e1 is :"<<e1<<std::endl;
    //std::cout<<"The value of the gigenvalue e2 is :"<<e2<<std::endl;
    // std::cout<<"The value of the gigenvalue e3 is :"<<e3<<std::endl;

    if ((e1 == e2) || (e2 == e3) || (e1 == e3)) {
        std::cerr << "The eigenvalue should not be equal!!!" << std::endl;
    }

    const double sum_of_eigenvalues = e1 + e2 + e3;
    double kOneThird = 1.0 / 3.0;

    if (sum_of_eigenvalues == 0.0) {
        std::cerr << "The sum of the eigenvalue is 0.0" << std::endl;
    }

    feature_vec.push_back((e1 - e2) / e1);//linearity
    feature_vec.push_back((e2 - e3) / e1);//planarity
    feature_vec.push_back(e3 / e1);//scattering
    //feature_vec.push_back(std::pow(e1 * e2 * e3, kOneThird));//omnivariance
    feature_vec.push_back((e1 - e3) / e1);//anisotropy
    //feature_vec.push_back((e1 * std::log(e1)) + (e2 * std::log(e2)) + (e3 * std::log(e3)));//eigen_entropy
    //feature_vec.push_back(e3 / sum_of_eigenvalues);//change_of_curvature
    //feature_vec.push_back( int (lable));

    Feature eigenvalue_feature;
//
//    vector<double>::iterator t;
//    for (t = feature_vec.begin(); t != feature_vec.end(); t++) {
//
//        std::cout <<std::fixed<< *t << " ";
//
//    }
//    std::cout<<int (lable)<<std::endl;



}


void consume_thread(){

  cerr<<"Now it runs in the consume_thread function."<<endl;
    while (true){

        std::unique_lock<std::mutex> lck(m_mux);
        if(cloud_vector.empty()) {
            lck.unlock();
            continue;
        }


        pcl::PointCloud<PointT> test_tmp;
        test_tmp=cloud_vector.front();


        cerr<<"The size of the test_res is :"<<test_tmp.size()<<endl;
        cloud_vector.pop_front();
        ROS_ERROR("CONSUME ONE PRODUCT");
        cerr<<"The size of the cloud_vector is :"<<cloud_vector.size()<<endl;
        lck.unlock();


        pcl::PointCloud<PclPoint>::Ptr origin_cloud (new pcl::PointCloud<PclPoint>);
    //   pcl::PointCloud<PointT>::Ptr origin_cloud (new pcl::PointCloud<PointT>);

        origin_cloud->points.resize (test_tmp.width * test_tmp.height);

        //origin_cloud=test_tmp;



        for (size_t i = 0; i < test_tmp.points.size(); ++i) {

            origin_cloud->points[i].x=test_tmp.points[i].x;
            origin_cloud->points[i].y=test_tmp.points[i].y;
            origin_cloud->points[i].z=test_tmp.points[i].z;
            origin_cloud->points[i].intensity=1200;
//
//            1100 is the wall
//            1200 is the target
//            1300 is the chair
//            1400 is the people
//            1500 is the bottle
//            1600 is the box
//            1700 is the flower
//            1800 is the garbage
//            1900 is the column
//
//            2000 is the diff small bottle
//            2100 is the diff flower
//            2200 is the diff garbage
//            2300 is the diff bottle
//            2400 is the diff chair


        }

      //  cerr<<"Success !!!"<<endl;


        ROS_ERROR("Now start recording the running time !");

        clock_t start, end;
        start = clock();

        //创建KdTreeFLANN对象，并把创建的点云设置为输入,创建一个searchPoint变量作为查询点
        pcl::KdTreeFLANN<PclPoint> kdtree;
        //设置搜索空间
        kdtree.setInputCloud(origin_cloud);
        //设置查询点并赋随机值

        //For every point we try to find its neighbor points.
        for (size_t i = 0; i < origin_cloud->points.size(); ++i) {
            PclPoint searchPoint;
            searchPoint.x = origin_cloud->points[i].x;
            searchPoint.y = origin_cloud->points[i].y;
            searchPoint.z = origin_cloud->points[i].z;
            searchPoint.intensity = origin_cloud->points[i].intensity;

            //This is the test of one point,try to find its neighbors
            //    searchPoint.x =  105.3;
            //    searchPoint.y =  114.28;
            //    searchPoint.z = -4.59;
            /**********************************************************************************
             下面的代码展示查找到给定的searchPoint的某一半径（随机产生）内所有近邻，重新定义两个向量
             pointIdxRadiusSearch  pointRadiusSquaredDistance来存储关于近邻的信息
             ********************************************************************************/
            // 半径 R内近邻搜索方法

            std::vector<int> pointIdxRadiusSearch;           //存储近邻索引
            std::vector<float> pointRadiusSquaredDistance;   //存储近邻对应距离的平方

            float radius = 0.6;//Here we set the radious is 0.4
            float local_density = 0;
            pcl::PointCloud<PclPoint> segment;

            if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >
                0) {

                PclPoint searchPoint_res;
                pcl::PointXYZ point_local;

                for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {

                    //
                    //                std::cout << " "  << origin_cloud->points[ pointIdxRadiusSearch[i] ].x
                    //                          << " " << origin_cloud->points[ pointIdxRadiusSearch[i] ].y
                    //                          << " " << origin_cloud->points[ pointIdxRadiusSearch[i] ].z
                    //                          << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
                    local_density += pointRadiusSquaredDistance[i];

                    searchPoint_res.x = origin_cloud->points[pointIdxRadiusSearch[i]].x;
                    searchPoint_res.y = origin_cloud->points[pointIdxRadiusSearch[i]].y;
                    searchPoint_res.z = origin_cloud->points[pointIdxRadiusSearch[i]].z;
                    searchPoint_res.intensity = origin_cloud->points[pointIdxRadiusSearch[i]].intensity;
                    segment.push_back(searchPoint_res);

                    point_local.x = origin_cloud->points[pointIdxRadiusSearch[i]].x;
                    point_local.y = origin_cloud->points[pointIdxRadiusSearch[i]].y;
                    point_local.z = origin_cloud->points[pointIdxRadiusSearch[i]].z;
                }
                local_density = local_density / pointIdxRadiusSearch.size();

            }

            int num_points = segment.points.size();//get the number of the points
            // if the point's neighborhoood points is two low ,we consider it must be a noise point
            if (num_points <= 2) {
                ;
            } else {

                EigenvalueBasedDescriptor(segment, local_density, searchPoint.intensity);

            }
        }


        end = clock();
        cout<<"Run time: "<<(double)(end - start) / CLOCKS_PER_SEC<<"S"<<endl;
        ROS_WARN("It takes several s to calculate the feattures!");





    }
}







int
main (int argc, char** argv)
{

    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");


    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/o3d3xx/camera/cloud", 1, cloud_cb);



    std::thread thread (&consume_thread);
    std::cout<<"Now it runs after the thread function !"<<std::endl;

    //创建ROS的发布节点
    //  pub = nh.advertise<sensor_msgs::PointCloud2> ("/chy/point", 1);

    ros::MultiThreadedSpinner spinner(2);  // extra thread so we can receive cloud messages while in the service call
    spinner.spin();

    thread.join();
    std::cout<<"Now it runs after the thread.join function !"<<std::endl;
    // 回调


    return (0);
}