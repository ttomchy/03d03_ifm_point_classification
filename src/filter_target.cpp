#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

int
main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);



    int j_num_wall =0;
    std::string ss1;


    while(j_num_wall<29) {

        char szName[100] = {'\0'};


        sprintf(szName,
        "/home/laptop2/work_space/intern_ws/o3d/test_ws/dataset/training/diff_scale/diff_scale_chair/8/chair%d.pcd",

                j_num_wall); //格式化输出文件名



        // Fill in the cloud data
        pcl::PCDReader reader;
        pcl::PCDWriter writer;
        // Replace the path below, with the path where you saved your file
        reader.read<pcl::PointXYZ>( szName,*cloud);

        std::cerr << "Cloud before filtering: " << std::endl;
       // std::cerr << *cloud << std::endl;

        // Create the filtering object
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud);
        sor.setMeanK (25);
        sor.setStddevMulThresh (1.0);
        sor.filter (*cloud_filtered);

        std::cerr << "Cloud after filtering: " << std::endl;
        std::cerr << *cloud_filtered << std::endl;


        std::stringstream ss;
        ss << "/home/laptop2/work_space/intern_ws/o3d/test_ws/dataset/training/diff_scale/diff_scale_chair/test/chair"
           << j_num_wall << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_filtered, false);






//        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
//        tree2->setInputCloud (cloud_filtered);
//
//        std::vector<pcl::PointIndices> cluster_indices;
//        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//        ec.setClusterTolerance (0.1); // 2cm
//        ec.setMinClusterSize (50);
//        ec.setMaxClusterSize (500000);
//        ec.setSearchMethod (tree2);
//        ec.setInputCloud (cloud_filtered);
//        ec.extract (cluster_indices);

//
//        int j = 0;
//        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.begin ()+1; ++it)
//        {
//            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
//            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//                cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
//            cloud_cluster->width = cloud_cluster->points.size ();
//            cloud_cluster->height = 1;
//            cloud_cluster->is_dense = true;
//
//            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
//
//            std::stringstream ss;
//            ss << "/home/laptop2/work_space/intern_ws/o3d/test_ws/dataset/training/diff_scale/diff_scale_chair/test/chair"
//               << j_num_wall << ".pcd";
//            writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);
//
//        }

        j_num_wall++;
   }





    return (0);
}
