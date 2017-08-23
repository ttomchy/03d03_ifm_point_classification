#include <iostream>
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





typedef pcl::PointXYZ PointT;


int j_num_wall = 0;
int j_num_not_wall = 0;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input) {

    // All the objects needed
    pcl::PCDReader reader;
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PCDWriter writer;
    pcl::ExtractIndices<PointT> extract;
    //pcl::ExtractIndices<pcl::PointXYZ> extract;
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
    sor.setStddevMulThresh(1.0);
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
    seg.setDistanceThreshold (0.1);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
    // std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;


    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

   //  Write the planar inliers to disk
     pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
      extract.filter (*cloud_plane);
     std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
      writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);


    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);



    pcl::PointCloud<PointT>::Ptr non_cloud_plane (new pcl::PointCloud<PointT> ());
    extract.filter (*non_cloud_plane);
    //  std::cerr << "PointCloud representing the non_cloud_plane planar component: " << non_cloud_plane->points.size () << " data points." << std::endl;
    // writer.write ("no-table_scene_mug_stereo_textured_plane.pcd", *cloud_filtered2, false);


    *cloud=*non_cloud_plane;
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
    tree2->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.1); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (500000);
    ec.setSearchMethod (tree2);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);


    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud->points[*pit]); //*

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::string ss1,ss2;

        if( cloud_cluster->points.size ()>2000 ){



//
//            double x_axis=0;
//
//            for(size_t i=0;i<cloud_cluster->points.size();++i)
//            {
//                x_axis+=cloud_cluster->points[i].x;
//            }
//
//            x_axis/=cloud_cluster->points.size();
//
//            if((x_axis<3)){

            ROS_WARN("Successful writing wall points!");
            ss1="./dataset/training/stair/test/";
            ss2="chair";
            // /home/laptop2/work_space/intern_ws/o3d/test_ws
            std::stringstream ss;
            ss <<ss1<<ss2  ;
            ss<<j_num_wall<< ".pcd";

            writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
            j_num_wall++;
            // }



        } else{
            //  ROS_ERROR("Successful writing non_wall points!");
            ss1="./dataset/training/stair/test/";
            ss2="flower";
            // /home/laptop2/work_space/intern_ws/o3d/test_ws
            std::stringstream ss;
            ss <<ss1<<ss2  ;
            ss<<j_num_not_wall<< ".pcd";

//            double x_axis=0;
//
//            for(size_t i=0;i<cloud_cluster->points.size();++i)
//            {
//                x_axis+=cloud_cluster->points[i].x;
//            }
//
//            x_axis/=cloud_cluster->points.size();

            //if((x_axis<4)&&(cloud_cluster->points.size()>490)){
            //    ROS_ERROR("Successful writing non_wall points!");

            writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
            j_num_not_wall++;


            //    }



        }

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

    //创建ROS的发布节点
    //  pub = nh.advertise<sensor_msgs::PointCloud2> ("/chy/point", 1);

    // 回调
    ros::spin ();

    return (0);
}