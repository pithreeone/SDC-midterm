#include <iostream>
#include <string>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;
using namespace ros;

const float range_resolution = 0.175;
Publisher radar_pub;
Publisher radar_original_pub;
double intensity_thres;

bool intensity_compare(pcl::PointXYZI a, pcl::PointXYZI b) 
{
    return a.intensity > b.intensity; 
}

pcl::PointCloud<pcl::PointXYZI>::Ptr create_radar_pc(Mat img)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_pc(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    
    /*TODO : Transform Polar Image to Cartisien Pointcloud*/
    int row = img.rows;   // 2856
    int col = img.cols;   // 400
    // col: angle
    for(int i=0; i<col; i++){
        static double PI = 3.1415926;
        double angle = (double) i/col*2*PI;
        // row: distance
        for(int j=4; j<row; j++){
            double distance = (j - 4) * range_resolution;
            pcl::PointXYZI point;
            point.x = distance * cos(-angle);
            point.y = distance * sin(-angle);
            point.z = 0;
            // std::cout << static_cast<float>(img.at<uchar>(j,i)) << std::endl;
            point.intensity = static_cast<float>(img.at<uchar>(j,i));
            if(point.intensity < intensity_thres){
                point.intensity = 0;
                continue;
            }
            new_pc->points.push_back(point);
        }
    }
    // sensor_msgs::PointCloud2 pc_msg;
    // pcl::toROSMsg(*new_pc, pc_msg);
    // pc_msg.header.stamp = ros::Time::now();
    // pc_msg.header.frame_id = "map";
    // radar_original_pub.publish(pc_msg);


    // pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    // sor.setInputCloud(new_pc);
    // sor.setMeanK(50); // Adjust the number of neighbors to consider
    // sor.setStddevMulThresh(1.0); // Adjust the standard deviation threshold
    // sor.filter(*cloud_filtered);


    // return cloud_filtered;
    return new_pc;
}

void radarCallback(const sensor_msgs::ImageConstPtr& msg)
{
    Mat img;
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(msg, "mono8");
    img = cv_ptr->image;
    pcl::PointCloud<pcl::PointXYZI>::Ptr radar_pc_ptr = create_radar_pc(img);
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(*radar_pc_ptr, pc_msg);
    pc_msg.header.stamp = ros::Time::now();
    pc_msg.header.frame_id = "base_link";
    radar_pub.publish(pc_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_polar_to_pointcloud");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    image_transport::ImageTransport it(nh);
    radar_pub = nh.advertise<sensor_msgs::PointCloud2>("/radar_pc", 1);
    radar_original_pub = nh.advertise<sensor_msgs::PointCloud2>("/radar_original_pc", 1);
    image_transport::Subscriber sub = it.subscribe("/Navtech/Polar", 1, radarCallback);
    
    nh_local.getParam("intensity_thres", intensity_thres);

    ros::spin();
    return 0;
}