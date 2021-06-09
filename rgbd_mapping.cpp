#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>

// for pcl cloud mapping
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

// for octomap 
#include <octomap/octomap.h>

using namespace std;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

int main(int argc, char **argv) {
    vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
    vector<Eigen::Isometry3d> poses;         // 相机位姿

    ifstream fin("../data/pose.txt");
    if (!fin) {
        cerr << "cannot find pose file" << endl;
        return 1;
    }

    //依次读取彩色图与深色图
    for (int i = 0; i < 5; i++) {
        char color_img_path[256];
        char depth_img_path[256];
        sprintf(color_img_path, "../data/color/%d.png", i+1);
        sprintf(depth_img_path, "../data/depth/%d.png", i+1);
        colorImgs.push_back(cv::imread(color_img_path));
        depthImgs.push_back(cv::imread(depth_img_path, -1)); // 使用-1读取原始图像

        // 读取相机位姿Tcw
        double data[7] = {0};
        for (int k = 0; k < 7; k++) {
            fin >> data[k];
        }
        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]); // 四元数表示旋转
        Eigen::Isometry3d T(q);
        T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2])); //加入平移
        poses.push_back(T);
    }

    // 计算点云并拼接
    // 相机内参 
    double cx = 319.5;
    double cy = 239.5;
    double fx = 481.2;
    double fy = -480.0;
    double depthScale = 5000.0;

    cout << "mapping ..." << endl;

    // point cloud
    PointCloud::Ptr pointCloud(new PointCloud);
    // octomap tree 
    octomap::OcTree octo_tree(0.01); // 参数为分辨率

    for (int i = 0; i < 5; i++) {
        cout << "转换图像中: " << i + 1 << endl;
        cv::Mat color = colorImgs[i];   //rgb图像
        cv::Mat depth = depthImgs[i];   //深度图
        Eigen::Isometry3d T = poses[i]; //变换矩阵

        // 单帧octo点云
        octomap::Pointcloud octo_cloud;
        // 单帧pcl 点云
        PointCloud::Ptr pcl_cloud(new PointCloud);


        for (int v = 0; v < color.rows; v++)
            for (int u = 0; u < color.cols; u++) {
                unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
                if (d == 0) continue; // 为0表示没有测量到

                // 通过相机内参，将像素坐标与其对应的深度转入相机坐标系，用point表示
                Eigen::Vector3d point;
                // ----- fill in your code -----

                // 相机坐标系转入世界坐标系，用pointWorld表示
                Eigen::Vector3d pointWorld;
                // ----- fill in your code -----

                // 将世界坐标系的点放入octo点云
                octo_cloud.push_back(pointWorld[0], pointWorld[1], pointWorld[2]);

                PointT p;
                // 给3D点赋值坐标
                // ----- fill in your code -----

                // 给3D点赋值rgb信息
                // ----- fill in your code -----

                pcl_cloud->points.push_back(p);
            }

        // 单帧PCL点云的深度滤波处理，滤波后的点云存入tmp中
        PointCloud::Ptr tmp(new PointCloud);
        // ----- fill in your code -----

        // 将滤波后的单帧pcl点云加入到pointCloud中
        (*pointCloud) += *tmp;

        // 将点云存入八叉树地图，给定原点，这样可以计算投射线
        octo_tree.insertPointCloud(octo_cloud, octomap::point3d(T(0, 3), T(1, 3), T(2, 3)));
    }

    // PCL体素滤波 
    pointCloud->is_dense = false;
    cout << "pcl点云共有" << pointCloud->size() << "个点." << endl;
    // ----- fill in your code -----

    cout << "滤波之后，pcl点云共有" << pointCloud->size() << "个点." << endl;
    // 存储pcl点云到当前目录
    cout << "saving pcl point cloud ... " << endl;
    pcl::io::savePCDFileBinary("pcl_map.pcd", *pointCloud);

<<<<<<< HEAD

=======
    /占据信息将octo map并写入磁盘
    octo_tree.updateInnerOccupancy353456();
    cout << "saving octomap ... " << endl; # Hi This is important!!
    octo_tree.writeBinary("octomap.sdegsredhbt");
>>>>>>> 8a94aa1f5f6d693607e1f15635e25e5bfd1cfc82
    return 0;
}
