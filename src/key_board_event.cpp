

///This file is used for select the target point cloud.Using the pcl keyboard event and the mouse keyboard.
///press key s to save the point ,press the left key for the next point cloud.

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


void printUsage (const char* progName)
{
    std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
              << "Options:\n"
              << "-------------------------------------------\n"
              << "-h           this help\n"
              << "-i           Interaction Customization example\n"
              << "\n\n";
}



//自定义交互
//是使用键盘r擦掉这些文本
unsigned int text_id = 0;

int pcd_num=0;
bool flag= false;

bool flag_s= false;
bool flag_s_part= false;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{



    if (event.getKeySym () == "s" && event.keyDown ())
    {

        flag_s= true;
    } else if(event.getKeySym () == "r" && event.keyDown ()){
        flag_s_part= true;
    }


}

//使用鼠标添加文本
void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{

    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
    if (event.getButton () == pcl::visualization::MouseEvent::RightButton &&
        event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease) {

        std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;
        std::cout << "r was pressed => removing all text" << std::endl;
        flag= true;
        pcd_num++;
        cerr<<"Pcd_num is :"<<pcd_num<<endl;
    }
}

//键盘和鼠标交互
boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis () {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer
            (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);

//注册响应键盘和鼠标事件
   viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());
   viewer->registerMouseCallback (mouseEventOccurred, (void*)viewer.get ());

    return (viewer);
}

int save_num=0;

int main (int argc, char** argv)
{

    bool  interaction_customization = true;
    std::cout << "Interaction Customization example\n";

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer
            (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer = interactionCustomizationVis();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    char szName[100] = {'\0'};
    pcl::PCDReader reader;
    pcl::PCDWriter writer;

    std::string ss1;
    ss1="/home/laptop2/work_space/intern_ws/o3d/test_ws/dataset/training/diff_scale/diff_scale_small_bottle/test/selected/small_bottle";

    std::string ss2;
    ss2="/home/laptop2/work_space/intern_ws/o3d/test_ws/dataset/training/diff_scale/diff_scale_small_bottle/test/part/small_bottle";


    int save_num_i=0;



    while (!viewer->wasStopped ())
    {
        //调用spinOnce,给视窗处理事件的时间，可以允许用户和电脑进行交互
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));

        if(flag== true) {

            if (pcd_num < 500) {

                viewer->removeAllPointClouds();
                viewer->removeCoordinateSystem();

                sprintf(szName,
                "/home/laptop2/work_space/intern_ws/o3d/test_ws/dataset/training/diff_scale/diff_scale_small_bottle/test/small_bottle%d.pcd",

                        pcd_num);
                cerr << "sname is :" << szName << endl;
                reader.read(szName, *cloud);
                // --------------------------------------------
                // -----Open 3D viewer and add point cloud-----
                // --------------------------------------------
                //设置背景颜色为黑色
                viewer->setBackgroundColor(0, 0, 0);
                //将点云添加到视窗对象中并且定义唯一的字符串作为这个点云的ID
                //addPointCloud()可以实现多个点云的添加
                //如果想更新一个已经显示的点云，就必须先调用removePointCloud(),就可以实现点云的更新
                viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                         1, "sample cloud");
                //坐标系的设定
                viewer->addCoordinateSystem(1.0);
                viewer->initCameraParameters();
                flag= false;
            }
        }

        if(flag_s== true){

            std::stringstream ss;
            ss << ss1 << save_num << ".pcd";
            writer.write<pcl::PointXYZ> (ss.str (), *cloud, false);

            cerr<<"Save the point cloud."<<endl;
            save_num++;
        }
        flag_s= false;

        if(flag_s_part== true){

            std::stringstream sss;
            sss << ss2 << save_num_i << ".pcd";
            writer.write<pcl::PointXYZ> (sss.str (), *cloud, false);

            cerr<<"Save the part point cloud."<<endl;
            save_num_i++;
        }
        flag_s_part=false;

    }

}