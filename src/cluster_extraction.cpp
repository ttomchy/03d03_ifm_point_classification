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
#include <string>

int
main (int argc, char** argv)
{
    // Read in the cloud data
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read ("no-table_scene_mug_stereo_textured_plane.pcd", *cloud);

    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
    std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

//    // Create the filtering object: downsample the dataset using a leaf size of 1cm
//    pcl::VoxelGrid<pcl::PointXYZ> vg;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//    vg.setInputCloud (cloud);
//    vg.setLeafSize (0.01f, 0.01f, 0.01f);
//    vg.filter (*cloud_filtered);
//    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*


    pcl::PCDWriter writer;

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.1); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (500000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::string ss1,ss2;

        if( cloud_cluster->points.size ()>700){
            ss1="./dataset/training/wall/";
            ss2="clouser_wall";
            // /home/laptop2/work_space/intern_ws/o3d/test_ws
            std::stringstream ss;
            ss <<ss1<<ss2  ;
            ss<<j<< ".pcd";

            writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        } else{
            ss1="./dataset/training/not_wall/";
            ss2="clouser_not_wall";
            // /home/laptop2/work_space/intern_ws/o3d/test_ws
            std::stringstream ss;
            ss <<ss1<<ss2  ;
            ss<<j<< ".pcd";

            writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*

        }

        j++;
    }

    return (0);
}
