#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>
#include <iostream>

ros::Publisher pub;

void box_cb (const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& box_array_msg)
{
    pcl::PointCloud<pcl::PointXYZ> myCloud;

    for(int i=0; i<box_array_msg->boxes.size();i++){
        std::cout<<"position"<<std::endl;
        std::cout<<box_array_msg->boxes[i].pose.position.x<<std::endl;
        std::cout<<box_array_msg->boxes[i].pose.position.y<<std::endl;
        std::cout<<box_array_msg->boxes[i].pose.position.z<<std::endl;
        std::cout<<"dimension"<<std::endl;
        std::cout<<box_array_msg->boxes[i].dimensions.x<<std::endl;
        std::cout<<box_array_msg->boxes[i].dimensions.y<<std::endl;
        std::cout<<box_array_msg->boxes[i].dimensions.z<<std::endl;

        double x_s = (box_array_msg->boxes[i].pose.position.x - box_array_msg->boxes[i].dimensions.x/2);
        double x_r = (box_array_msg->boxes[i].pose.position.x + box_array_msg->boxes[i].dimensions.x/2);
        double y_s = (box_array_msg->boxes[i].pose.position.y - box_array_msg->boxes[i].dimensions.y/2);
        double y_r = (box_array_msg->boxes[i].pose.position.y + box_array_msg->boxes[i].dimensions.y/2);

        for(double x=x_s; x<x_r;x=x+0.1){
            for(double y=y_s;y<y_r;y=y+0.1){
                pcl::PointXYZ newPoint;
                newPoint.x = x;
                newPoint.y = y;
                newPoint.z = box_array_msg->boxes[i].pose.position.z;
                myCloud.points.push_back(newPoint);
            }
        }
    }

    pcl::PCLPointCloud2 pcl_pc;
    sensor_msgs::PointCloud2 ros_output;
    

    pcl::toPCLPointCloud2(myCloud, pcl_pc);
    pcl_conversions::fromPCL(pcl_pc, ros_output);
    
    ros_output.header.stamp = ros::Time::now();
    ros_output.header.frame_id = "map";

    pub.publish(ros_output);

}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "box_pc");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe("/zedm/zed_node/point_cloud/cloud_registered", 1, cloud_cb);
    ros::Subscriber sub = nh.subscribe("/boxes_filtered", 1, box_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/pub_box_pc", 1);

    // Spin
    ros::spin ();
}