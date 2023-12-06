#include <string>
#include <fstream>
#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl_ros/transforms.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>

using namespace std;
class Localizer
{
private:
    ros::NodeHandle _nh;
    ros::NodeHandle _nh_local;

    ros::Subscriber radar_pc_sub;
    ros::Subscriber map_sub;
    ros::Subscriber gps_sub;

    ros::Publisher radar_pc_pub;
    ros::Publisher radar_pose_pub;
    ros::Publisher path_pub;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_pc;
    nav_msgs::Path path;
    tf::TransformBroadcaster br;
    
    std::string save_path;
    std::ofstream file;

    float pose_x;
    float pose_y;
    float pose_yaw;
    float gps_x;
    float gps_y;
    float gps_yaw;

    int seq = 0;
    int max_iter;
    float epsilon1;
    float epsilon2;
    float correspond;

    Eigen::Matrix4f init_guess;

    bool map_ready = false;
    bool gps_ready = false;
    bool initialized = false;

    // ros-parameters
    int filter_type;
    int scan_matching_type;
    double lpf_gain;
    double lpf_angle_gain;

    // KF-varibales
    Eigen::Matrix<double, 5, 1> state;
    Eigen::Matrix<double, 5, 5> cov_mat;
    Eigen::Matrix<double, 5, 5> A;
    Eigen::Matrix<double, 5, 5> C;
    Eigen::Matrix<double, 5, 5> Q;
    Eigen::Matrix<double, 5, 5> R;

public:
    Localizer(ros::NodeHandle nh, ros::NodeHandle nh_local) : map_pc(new pcl::PointCloud<pcl::PointXYZI>)
    {
        map_ready = false;
        gps_ready = false;
        
        _nh = nh;
        _nh_local = nh_local;
        _nh.param<string>("/save_path", save_path, "/Default/path");
        _nh_local.param<double>("low_pass_filter_gain", lpf_gain, 0.5);
        _nh_local.param<double>("low_pass_filter_angle_gain", lpf_angle_gain, 0.5);
        _nh_local.param<int>("filter_type", filter_type, 0);
        _nh_local.param<int>("scan_matching_type", scan_matching_type, 0);
        // ROS_INFO("low_pass_filter_angle_gain:%f", lpf_angle_gain);
        

        init_guess.setIdentity();
        file.open(save_path);
        file << "id,x,y,yaw\n";

        radar_pc_sub = _nh.subscribe("/radar_pc", 400, &Localizer::radar_pc_callback, this);
        map_sub = _nh.subscribe("/map_pc", 1, &Localizer::map_callback, this);
        gps_sub = _nh.subscribe("/gps", 1, &Localizer::gps_callback, this);

        radar_pc_pub = _nh.advertise<sensor_msgs::PointCloud2>("/tranformed_radar_pc", 1);
        radar_pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("/tranformed_radar_pose", 1);
        path_pub = _nh.advertise<nav_msgs::Path>("/localization_path", 1);

        // Initialize-KF-variables
        A << 1, 0, 0, 1, 0,
             0, 1, 0, 0, 1,
             0, 0, 1, 0, 0,
             0, 0, 0, 1, 0,
             0, 0, 0, 0, 1;
        
        C << 1, 0, 0, 0, 0,
             0, 1, 0, 0, 0,
             0, 0, 1, 0, 0,
             0, 0, 0, 1, 0,
             0, 0, 0, 0, 1;
        
        Q << 1, 0, 0, 0, 0,
             0, 1, 0, 0, 0,
             0, 0, 1, 0, 0,
             0, 0, 0, 1, 0,
             0, 0, 0, 0, 1;
        
        // rosbag1
        // R <<  .6,  0,  0,  0,  0,
        //       0,  .6,  0,  0,  0,
        //       0,  0,  .58,  0,  0,
        //       0,  0,  0, .5,  0, 
        //       0,  0,  0,  0, .5;

        // rosbag2
        R <<  .6,  0,  0,  0,  0,
              0,  .6,  0,  0,  0,
              0,  0,  .01,  0,  0,
              0,  0,  0, .5,  0, 
              0,  0,  0,  0, .5;
        
        cov_mat << .1,  0,  0,  0,  0,
                    0, .1,  0,  0,  0,
                    0,  0, .1,  0,  0,
                    0,  0,  0, .1,  0,
                    0,  0,  0,  0, .1;
    }

    ~Localizer()
    {
        ROS_WARN("Exit Localization");
        file.close();
    }

    void gps_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        ROS_WARN("Got GPS data");
        gps_x = msg->pose.position.x;
        gps_y = msg->pose.position.y;
        tf::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );
        tf::Matrix3x3 m(q);
        double r, p, yaw;
        m.getRPY(r, p, yaw);
        gps_yaw = yaw;
        if(!gps_ready)
        {
            pose_x = gps_x;
            pose_y = gps_y;
            pose_yaw = gps_yaw;
            gps_ready = true;
        }
    }

    void map_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        ROS_WARN("Got Map Pointcloud");
        pcl::fromROSMsg(*msg, *map_pc);
        map_ready = true;
    }

    void print4x4Matrix (const Eigen::Matrix4f & matrix)
    {
        printf ("Rotation matrix :\n");
        printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
        printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
        printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
        printf ("Translation vector :\n");
        printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
    }

    void radar_pc_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        ROS_WARN("Got Radar Pointcloud");
        pcl::PointCloud<pcl::PointXYZI>::Ptr radar_pc(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_pc(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *radar_pc);
        ROS_INFO("point size: %d", radar_pc->width);

        while(!(map_ready && gps_ready))
        { 
            ROS_WARN("Wait for map and gps ready");
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }

        if(!initialized)
        {
            /*TODO : Initialize initial guess*/
            pose_x = gps_x;
            pose_y = gps_y;
            pose_yaw = gps_yaw;
            initialized = true;
        }

        /*TODO : Implenment any scan matching base on initial guess, ICP, NDT, etc. */
        /*TODO : Assign the result to pose_x, pose_y, pose_yaw */
        /*TODO : Use result as next time initial guess */
        Eigen::Matrix4f transformationMatrix;
        switch(scan_matching_type){
            case 0: {
                // // Method1: pcl
                pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
                icp.setInputSource(radar_pc);
                icp.setInputTarget(map_pc);

                // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
                icp.setMaxCorrespondenceDistance(5);
                // Set the maximum number of iterations (criterion 1)
                icp.setMaximumIterations(100);
                // Set the transformation epsilon (criterion 2)
                icp.setTransformationEpsilon(1e-8);
                // Set the euclidean distance difference epsilon (criterion 3)
                icp.setEuclideanFitnessEpsilon(1e-8);

                // set the initial guess
                Eigen::Matrix4f initialGuess = Eigen::Matrix4f::Identity();

                if(seq == 0){
                    state << pose_x, pose_y, pose_yaw, 0, 0;
                }
                
                // // KF - predict
                state = A * state;
                cov_mat = A * cov_mat * A.transpose() + Q;
                // Set translation values in the matrix
                initialGuess(0, 3) = state(0, 0);  // x translation
                initialGuess(1, 3) = state(1, 0);  // y translation
                initialGuess.block<2, 2>(0, 0) << cos(state(2, 0)), -sin(state(2, 0)), sin(state(2, 0)), cos(state(2, 0));


                // // Set translation values in the matrix
                // initialGuess(0, 3) = pose_x;  // x translation
                // initialGuess(1, 3) = pose_y;  // y translation
                // initialGuess.block<2, 2>(0, 0) << cos(pose_yaw), -sin(pose_yaw), sin(pose_yaw), cos(pose_yaw);
                // Perform the alignment
                icp.align(*output_pc, initialGuess);
                // Obtain the transformation that aligned cloud_source to cloud_source_registered
                transformationMatrix = icp.getFinalTransformation();   

                //Return the state of convergence after the last align run. 
                //If the two PointClouds align correctly then icp.hasConverged() = 1 (true). 
                ROS_INFO("has converged: %d\n", icp.hasConverged());
                ROS_INFO("score: %f", icp.getFitnessScore());
                break;
            }
            case 1: {
                // // Method2: NDT
                // Create NDT object
                pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
                ndt.setTransformationEpsilon(1e-4);
                ndt.setMaximumIterations(10);
                ndt.setStepSize(10);
                ndt.setResolution(10.0);

                // Set input clouds
                ndt.setInputSource(radar_pc);
                ndt.setInputTarget(map_pc);

                // set the initial guess
                Eigen::Matrix4f initialGuess = Eigen::Matrix4f::Identity();

                // Set translation values in the matrix
                initialGuess(0, 3) = pose_x;  // x translation
                initialGuess(1, 3) = pose_y;  // y translation
                initialGuess.block<2, 2>(0, 0) << cos(pose_yaw), -sin(pose_yaw), sin(pose_yaw), cos(pose_yaw);

                // Perform registration
                pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                ndt.align(*output_cloud, initialGuess);

                // Output transformation
                transformationMatrix = ndt.getFinalTransformation();
                break;
            }
        }

        // print4x4Matrix(transformationMatrix);
        // Extract the translation vector from the last column
        double temp_pose_x, temp_pose_y, temp_pose_yaw;
        Eigen::Vector3f translationVector = transformationMatrix.block<3, 1>(0, 3);
        temp_pose_x = translationVector[0];
        temp_pose_y = translationVector[1];

        // Extract the 3x3 rotation matrix from the upper-left corner
        Eigen::Matrix3f rotationMatrix = transformationMatrix.block<3, 3>(0, 0);

        // Extract RPY angles
        Eigen::Vector3f rpyAngles = rotationMatrix.eulerAngles(0, 1, 2); // Order of rotations: XYZ
        temp_pose_yaw = rpyAngles[2];
        
        static double temp_pose_x_before, temp_pose_y_before;
        switch(filter_type){
            case 0: {
                // do the low-pass filter
                if(seq != 0){
                    pose_x = pose_x + lpf_gain * (temp_pose_x - pose_x);
                    pose_y = pose_y + lpf_gain * (temp_pose_y - pose_y);
                    pose_yaw = pose_yaw + lpf_angle_gain * (temp_pose_yaw - pose_yaw);    
                }else{
                    pose_x = temp_pose_x;
                    pose_y = temp_pose_y;
                    pose_yaw = temp_pose_yaw;
                }

                break;
            }
            case 1:{
                Eigen::Matrix<double, 5, 1> measurement;
                if(seq == 0){
                    // state << temp_pose_x, temp_pose_y, temp_pose_yaw, 0, 0;
                    measurement << temp_pose_x, temp_pose_y, temp_pose_yaw, 0, 0;
                }else{
                    measurement << temp_pose_x, temp_pose_y, temp_pose_yaw, (temp_pose_x - temp_pose_x_before), (temp_pose_y - temp_pose_y_before);
                }
                // // KF - predict
                // state = A * state;
                // cov_mat = A * cov_mat * A.transpose() + Q;

                // KF - update              
                Eigen::Matrix<double, 5, 5> K;
                K = cov_mat * C.transpose() * (C * cov_mat * C.transpose() + R).inverse();
                state = state + K * (measurement - C * state);
                Eigen::Matrix<double, 5, 5> identityMatrix;
                identityMatrix << 1, 0, 0, 0, 0,
                                  0, 1, 0, 0, 0,
                                  0, 0, 1, 0, 0,
                                  0, 0, 0, 1, 0,
                                  0, 0, 0, 0, 1;
                cov_mat = (identityMatrix - K * C) * cov_mat;
                pose_x = state(0, 0);
                pose_y = state(1, 0);
                pose_yaw = state(2, 0);
                ROS_INFO("Kalman-Filter-State: x: %f, y: %f, yaw: %f, vx: %f, vy: %f", state(0, 0), state(1, 0), state(2, 0), state(3, 0), state(4, 0));
                break;
            }
        }
        temp_pose_x_before = temp_pose_x;
        temp_pose_y_before = temp_pose_y;

        // ROS_INFO("lpg_gain:%f", lpf_gain);
        if(seq == 0){
            pose_x = temp_pose_x;
            pose_y = temp_pose_y;
            pose_yaw = temp_pose_yaw;
        }
        tf_brocaster(pose_x, pose_y, pose_yaw);
        radar_pose_publisher(pose_x, pose_y, pose_yaw);

        sensor_msgs::PointCloud2 radar_pc_msg;
        pcl::toROSMsg(*radar_pc, radar_pc_msg);
        radar_pc_msg.header.stamp = ros::Time::now();
        radar_pc_msg.header.frame_id = "base_link";
        radar_pc_pub.publish(radar_pc_msg);
        ROS_INFO("Publish transformed pc");
        ROS_INFO("[seq %d] x:%.3f, y:%.3f, yaw:%.3f\n", seq, pose_x, pose_y, pose_yaw);

        file << seq << ",";
        file << pose_x << ",";
        file << pose_y << ",";
        file << pose_yaw << "\n";

        seq++;
    }

    void radar_pose_publisher(float x, float y, float yaw)
    {
        geometry_msgs::PoseStamped pose;
        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(0, 0, yaw);
        myQuaternion.normalize();

        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0;

        pose.pose.orientation.x = myQuaternion.getX();
        pose.pose.orientation.y = myQuaternion.getY();
        pose.pose.orientation.z = myQuaternion.getZ();
        pose.pose.orientation.w = myQuaternion.getW();
        radar_pose_pub.publish(pose);
        
        path.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        path.poses.push_back(pose);
        path_pub.publish(path);
    }

    void tf_brocaster(float x, float y, float yaw)
    {  
        ROS_INFO("Update map to baselink");
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(x, y, 0) );
        tf::Quaternion q;
        q.setRPY(0, 0, yaw);
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
    }

    void set_init_guess(float x, float y, float yaw)
    {
        init_guess(0, 0) = cos(yaw);
        init_guess(0, 1) = -sin(yaw);
        init_guess(0, 3) = x;

        init_guess(1, 0) = sin(yaw);
        init_guess(1, 1) = -sin(yaw);
        init_guess(1, 3) = y;
    }
};


int main(int argc, char** argv) 
{
    ros::init (argc, argv, "localizer");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    Localizer Localizer(nh, nh_local);

    ros::spin();
    return 0;
}